/* drivers/video/msm/msm_fb.c
 *
 * Core MSM framebuffer driver.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/msm_mdp.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <mach/msm_fb.h>
#include <mach/board.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_FB_MSM_REFRESH
#include <linux/sched.h>
#include <linux/kthread.h>
#endif 

#define MSM_FB_REFRESH_RATE 25

#define PRINT_FPS 0
#define PRINT_BLIT_TIME 0

#define SLEEPING 0x4
#define UPDATING 0x3
#define FULL_UPDATE_DONE 0x2
#define WAKING 0x1
#define AWAKE 0x0

#define NONE 0
#define SUSPEND_RESUME 0x1
#define FPS 0x2
#define BLIT_TIME 0x4
#define SHOW_UPDATES 0x8

#define DLOG(mask, fmt, args...) \
do { \
	if (msmfb_debug_mask & mask) \
		printk(KERN_INFO "msmfb: "fmt, ##args); \
} while (0)

#define BITS_PER_PIXEL(info) (info->fb->var.bits_per_pixel)
#define BYTES_PER_PIXEL(info) (info->fb->var.bits_per_pixel >> 3)

static int msmfb_debug_mask;
module_param_named(msmfb_debug_mask, msmfb_debug_mask, int,
		   S_IRUGO | S_IWUSR | S_IWGRP);

struct mdp_device *mdp;

struct msmfb_info {
	struct fb_info *fb;
	struct msm_panel_data *panel;
	int xres;
	int yres;
	unsigned output_format;
	unsigned yoffset;
	unsigned frame_requested;
	unsigned frame_done;
	int sleeping;
	unsigned update_frame;
	struct {
		int left;
		int top;
		int eright; /* exclusive */
		int ebottom; /* exclusive */
	} update_info;
	char *black;

	struct early_suspend earlier_suspend;
	struct early_suspend early_suspend;
	struct wake_lock idle_lock;
	spinlock_t update_lock;
	struct mutex panel_init_lock;
	wait_queue_head_t frame_wq;
	struct workqueue_struct *resume_workqueue;
	struct work_struct resume_work;
	struct msmfb_callback dma_callback;
	struct msmfb_callback vsync_callback;
	struct hrtimer fake_vsync;
	ktime_t vsync_request_time;
};

static int msmfb_open(struct fb_info *info, int user)
{
	return 0;
}

static int msmfb_release(struct fb_info *info, int user)
{
	return 0;
}

#ifdef CONFIG_FB_MSM_REFRESH
static void msmfb_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom);

struct task_struct *msmfb_thread_task = NULL;

static int msmfb_refresh_thread(void *v)
{
	struct fb_info *fb;
	struct completion exit;
	siginfo_t info;

	daemonize("msmfb_refreshd");
	allow_signal(SIGKILL);

	init_completion(&exit);
	
	while (true){
		if (signal_pending(current)){
			if (dequeue_signal_lock(current,
					&current->blocked, 
					&info) == SIGKILL)
				goto die;
		}
		else{
			msleep(MSM_FB_REFRESH_RATE);
			
			if (num_registered_fb > 0) {
				fb = registered_fb[0];
				msmfb_update(fb, 0, 0, 
					     fb->var.xres, fb->var.yres);
			}
		}	
	}
die:
	complete_and_exit(&exit,0);
}

static void start_msmfb_refresh_thread_locked(void){
	void *v = NULL;
	if (msmfb_thread_task) {
		return;
	}
	msmfb_thread_task = kthread_create(msmfb_refresh_thread, v, "msmfb_refreshd");	
	wake_up_process(msmfb_thread_task);
}

static void stop_msmfb_refresh_thread_locked(void){
	if (msmfb_thread_task) {
		send_sig(SIGKILL, msmfb_thread_task, 0);
		msmfb_thread_task = NULL;
	}
}
#endif

/* Called from dma interrupt handler, must not sleep */
static void msmfb_handle_dma_interrupt(struct msmfb_callback *callback)
{
	unsigned long irq_flags;
	struct msmfb_info *msmfb  = container_of(callback, struct msmfb_info,
					       dma_callback);

	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	msmfb->frame_done = msmfb->frame_requested;
	if (msmfb->sleeping == UPDATING &&
	    msmfb->frame_done == msmfb->update_frame) {
		DLOG(SUSPEND_RESUME, "full update completed\n");
		queue_work(msmfb->resume_workqueue, &msmfb->resume_work);
	}
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	wake_up(&msmfb->frame_wq);
}

static int msmfb_start_dma(struct msmfb_info *msmfb)
{
	uint32_t x, y, w, h;
	unsigned addr;
	unsigned long irq_flags;
	uint32_t yoffset;
	s64 time_since_request;
	struct msm_panel_data *panel = msmfb->panel;

	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	time_since_request = ktime_to_ns(ktime_sub(ktime_get(),
			     msmfb->vsync_request_time));
	if (time_since_request > 20 * NSEC_PER_MSEC) {
		uint32_t us;
		us = do_div(time_since_request, NSEC_PER_MSEC) / NSEC_PER_USEC;
		dev_dbg(&mdp->dev, "msmfb_start_dma %lld.%03u ms after vsync "
			"request\n", time_since_request, us);
	}
	if (msmfb->frame_done == msmfb->frame_requested) {
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
		return -1;
	}
	if (msmfb->sleeping == SLEEPING) {
		DLOG(SUSPEND_RESUME, "tried to start dma while asleep\n");
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
		return -1;
	}
	x = msmfb->update_info.left;
	y = msmfb->update_info.top;
	w = msmfb->update_info.eright - x;
	h = msmfb->update_info.ebottom - y;
	yoffset = msmfb->yoffset;
	msmfb->update_info.left = msmfb->xres + 1;
	msmfb->update_info.top = msmfb->yres + 1;
	msmfb->update_info.eright = 0;
	msmfb->update_info.ebottom = 0;
	if (unlikely(w > msmfb->xres || h > msmfb->yres ||
		     w == 0 || h == 0)) {
		dev_dbg(&mdp->dev, "invalid update: %d %d %d "
				"%d\n", x, y, w, h);
		msmfb->frame_done = msmfb->frame_requested;
		goto error;
	}
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);

	addr = ((msmfb->xres * (yoffset + y) + x) * BYTES_PER_PIXEL(msmfb));
	mdp->dma(mdp, addr + msmfb->fb->fix.smem_start,
		 msmfb->xres * BYTES_PER_PIXEL(msmfb), w, h, x, y,
		 &msmfb->dma_callback,
		 panel->interface_type);
	return 0;
error:
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	/* some clients need to clear their vsync interrupt */
	if (panel->clear_vsync)
		panel->clear_vsync(panel);
	wake_up(&msmfb->frame_wq);
	return 0;
}

/* Called from esync interrupt handler, must not sleep */
static void msmfb_handle_vsync_interrupt(struct msmfb_callback *callback)
{
	struct msmfb_info *msmfb = container_of(callback, struct msmfb_info,
					       vsync_callback);
	wake_unlock(&msmfb->idle_lock);
	msmfb_start_dma(msmfb);
}

static enum hrtimer_restart msmfb_fake_vsync(struct hrtimer *timer)
{
	struct msmfb_info *msmfb  = container_of(timer, struct msmfb_info,
					       fake_vsync);
	msmfb_start_dma(msmfb);
	return HRTIMER_NORESTART;
}

static void msmfb_pan_update(struct fb_info *info, uint32_t left, uint32_t top,
			     uint32_t eright, uint32_t ebottom,
			     uint32_t yoffset, int pan_display)
{
	struct msmfb_info *msmfb = info->par;
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;
	int sleeping;
	int retry = 1;

	DLOG(SHOW_UPDATES, "update %d %d %d %d %d %d\n",
		left, top, eright, ebottom, yoffset, pan_display);
restart:
	spin_lock_irqsave(&msmfb->update_lock, irq_flags);

	/* if we are sleeping, on a pan_display wait 10ms (to throttle back
	 * drawing otherwise return */
	if (msmfb->sleeping == SLEEPING) {
		DLOG(SUSPEND_RESUME, "drawing while asleep\n");
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
		if (pan_display)
			wait_event_interruptible_timeout(msmfb->frame_wq,
				msmfb->sleeping != SLEEPING, HZ/10);
		return;
	}

	sleeping = msmfb->sleeping;
	/* on a full update, if the last frame has not completed, wait for it */
	if ((pan_display && msmfb->frame_requested != msmfb->frame_done) ||
			    sleeping == UPDATING) {
		int ret;
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
		ret = wait_event_interruptible_timeout(msmfb->frame_wq,
			msmfb->frame_done == msmfb->frame_requested &&
			msmfb->sleeping != UPDATING, 5 * HZ);
		if (ret <= 0 && (msmfb->frame_requested != msmfb->frame_done ||
				 msmfb->sleeping == UPDATING)) {
			if (retry && panel->request_vsync &&
			    (sleeping == AWAKE)) {
				wake_lock_timeout(&msmfb->idle_lock, HZ/4);
				panel->request_vsync(panel,
					&msmfb->vsync_callback);
				retry = 0;
				dev_dbg(&mdp->dev, "msmfb_pan_display timeout "
					"rerequest vsync\n");
			} else {
				dev_dbg(&mdp->dev, "msmfb_pan_display timeout "
					"waiting for frame start, %d %d\n",
					msmfb->frame_requested,
					msmfb->frame_done);
				return;
			}
		}
		goto restart;
	}

	msmfb->frame_requested++;
	/* if necessary, update the y offset, if this is the
	 * first full update on resume, set the sleeping state */
	if (pan_display) {
		msmfb->yoffset = yoffset;
		if (left == 0 && top == 0 && eright == info->var.xres &&
		    ebottom == info->var.yres) {
			if (sleeping == WAKING) {
				msmfb->update_frame = msmfb->frame_requested;
				DLOG(SUSPEND_RESUME, "full update starting\n");
				msmfb->sleeping = UPDATING;
			}
		}
	}

	/* set the update request */
	if (left < msmfb->update_info.left)
		msmfb->update_info.left = left;
	if (top < msmfb->update_info.top)
		msmfb->update_info.top = top;
	if (eright > msmfb->update_info.eright)
		msmfb->update_info.eright = eright;
	if (ebottom > msmfb->update_info.ebottom)
		msmfb->update_info.ebottom = ebottom;
	DLOG(SHOW_UPDATES, "update queued %d %d %d %d %d\n",
		msmfb->update_info.left, msmfb->update_info.top,
		msmfb->update_info.eright, msmfb->update_info.ebottom,
		msmfb->yoffset);
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);

	/* if the panel is all the way on wait for vsync, otherwise sleep
	 * for 16 ms (long enough for the dma to panel) and then begin dma */
	msmfb->vsync_request_time = ktime_get();
	if (panel->request_vsync && (sleeping == AWAKE)) {
		wake_lock_timeout(&msmfb->idle_lock, HZ/4);
		panel->request_vsync(panel, &msmfb->vsync_callback);
	} else {
		if (!hrtimer_active(&msmfb->fake_vsync)) {
			hrtimer_start(&msmfb->fake_vsync,
				      ktime_set(0, NSEC_PER_SEC/60),
				      HRTIMER_MODE_REL);
		}
	}
}

static void msmfb_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom)
{
	msmfb_pan_update(info, left, top, eright, ebottom, 0, 0);
}

static void power_on_panel(struct work_struct *work)
{
	struct msmfb_info *msmfb =
		container_of(work, struct msmfb_info, resume_work);
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;

	mutex_lock(&msmfb->panel_init_lock);
	DLOG(SUSPEND_RESUME, "turning on panel\n");
	if (msmfb->sleeping == UPDATING) {
		wake_lock_timeout(&msmfb->idle_lock, HZ);
		if (panel->unblank(panel)) {
			printk(KERN_INFO "msmfb: panel unblank failed,"
			       "not starting drawing\n");
			goto error;
		}
		wake_unlock(&msmfb->idle_lock);
		spin_lock_irqsave(&msmfb->update_lock, irq_flags);
		msmfb->sleeping = AWAKE;
		wake_up(&msmfb->frame_wq);
		spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	}
error:
	mutex_unlock(&msmfb->panel_init_lock);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/* turn off the panel */
static void msmfb_earlier_suspend(struct early_suspend *h)
{
	struct msmfb_info *msmfb = container_of(h, struct msmfb_info,
						earlier_suspend);
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;

	mutex_lock(&msmfb->panel_init_lock);
	msmfb->sleeping = SLEEPING;
	wake_up(&msmfb->frame_wq);
	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);
	wait_event_timeout(msmfb->frame_wq,
			   msmfb->frame_requested == msmfb->frame_done, HZ/10);

	mdp->dma(mdp, virt_to_phys(msmfb->black), 0,
		 msmfb->fb->var.xres, msmfb->fb->var.yres, 0, 0,
		 NULL, panel->interface_type);
	mdp->dma_wait(mdp, panel->interface_type);

	/* turn off the panel */
	panel->blank(panel);
}

static void msmfb_suspend(struct early_suspend *h)
{
	struct msmfb_info *msmfb = container_of(h, struct msmfb_info,
						early_suspend);
	struct msm_panel_data *panel = msmfb->panel;
	/* suspend the panel */
	panel->suspend(panel);
	mutex_unlock(&msmfb->panel_init_lock);
}

static void msmfb_resume(struct early_suspend *h)
{
	struct msmfb_info *msmfb = container_of(h, struct msmfb_info,
						early_suspend);
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;
	struct fb_info *info = msmfb->fb;

	if (panel->resume(panel)) {
		printk(KERN_INFO "msmfb: panel resume failed, not resuming "
		       "fb\n");
		return;
	}
	spin_lock_irqsave(&msmfb->update_lock, irq_flags);
	msmfb->frame_requested = msmfb->frame_done = msmfb->update_frame = 0;
	msmfb->sleeping = WAKING;
	DLOG(SUSPEND_RESUME, "ready, waiting for full update\n");
	spin_unlock_irqrestore(&msmfb->update_lock, irq_flags);

	/* Launch a manual pan_update (otherwise depends on
	 * fbmem ioctl FBIOPUT_VSCREENINFO call). */
	msmfb_pan_update(info, 0, 0, info->var.xres, info->var.yres,
									info->var.yoffset, 1);
}
#endif


#ifdef CONFIG_MSMFB_FBIOBLANK
static int msmfb_blank(int blank_mode,struct fb_info *info){
	struct msmfb_info *msmfb = info->par;
	struct msm_panel_data *panel = msmfb->panel;
	unsigned long irq_flags;

	printk("%s(%d)\n", __func__, blank_mode != FB_BLANK_UNBLANK);
#ifdef CONFIG_FB_MSM_REFRESH
	mutex_lock(&msmfb->panel_init_lock);
	if (blank_mode == FB_BLANK_UNBLANK){
		start_msmfb_refresh_thread_locked();
	}
	else if(blank_mode == FB_BLANK_POWERDOWN){
		stop_msmfb_refresh_thread_locked();
	}
	mutex_unlock(&msmfb->panel_init_lock);
#endif
	return 0;
}
#endif

static int msmfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u32 size;

	if ((var->xres != info->var.xres) ||
	    (var->yres != info->var.yres) ||
	    (var->xoffset != info->var.xoffset) ||
	    (mdp->check_output_format(mdp, var->bits_per_pixel)) ||
	    (var->grayscale != info->var.grayscale))
		 return -EINVAL;

	size = var->xres_virtual * var->yres_virtual *
		(var->bits_per_pixel >> 3);
	if (size > info->fix.smem_len)
		return -EINVAL;
	return 0;
}

static int msmfb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;

	/* we only support RGB ordering for now */
	if (var->bits_per_pixel == 32 || var->bits_per_pixel == 24) {
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
	} else if (var->bits_per_pixel == 16) {
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
	} else
		return -1;
	mdp->set_output_format(mdp, var->bits_per_pixel);
	fix->line_length = var->xres * var->bits_per_pixel / 8;

	return 0;
}

int msmfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct msmfb_info *msmfb = info->par;
	struct msm_panel_data *panel = msmfb->panel;

	/* "UPDT" */
	if ((panel->caps & MSMFB_CAP_PARTIAL_UPDATES) &&
	    (var->reserved[0] == 0x54445055)) {
		msmfb_pan_update(info, var->reserved[1] & 0xffff,
				 var->reserved[1] >> 16,
				 var->reserved[2] & 0xffff,
				 var->reserved[2] >> 16, var->yoffset, 1);
	} else {
		msmfb_pan_update(info, 0, 0, info->var.xres, info->var.yres,
				 var->yoffset, 1);
	}
	return 0;
}

static void msmfb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	cfb_fillrect(p, rect);
	msmfb_update(p, rect->dx, rect->dy, rect->dx + rect->width,
		     rect->dy + rect->height);
}

static void msmfb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	cfb_copyarea(p, area);
	msmfb_update(p, area->dx, area->dy, area->dx + area->width,
		     area->dy + area->height);
}

static void msmfb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	cfb_imageblit(p, image);
	msmfb_update(p, image->dx, image->dy, image->dx + image->width,
		     image->dy + image->height);
}


static int msmfb_blit(struct fb_info *info,
		      void __user *p)
{
	struct mdp_blit_req req;
	struct mdp_blit_req_list req_list;
	int i;
	int ret;

	if (copy_from_user(&req_list, p, sizeof(req_list)))
		return -EFAULT;

	for (i = 0; i < req_list.count; i++) {
		struct mdp_blit_req_list *list =
			(struct mdp_blit_req_list *)p;
		if (copy_from_user(&req, &list->req[i], sizeof(req)))
			return -EFAULT;
		ret = mdp->blit(mdp, info, &req);
		if (ret)
			return ret;
	}
	return 0;
}


DEFINE_MUTEX(mdp_ppp_lock);

static int msmfb_ioctl(struct fb_info *p, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret;

	switch (cmd) {
	case MSMFB_GRP_DISP:
		mdp->set_grp_disp(mdp, arg);
		break;
	case MSMFB_BLIT:
		ret = msmfb_blit(p, argp);
		if (ret)
			return ret;
		break;

	default:
			printk(KERN_INFO "msmfb unknown ioctl: %d\n", cmd);
			return -EINVAL;
	}
	return 0;
}

static struct fb_ops msmfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = msmfb_open,
	.fb_release = msmfb_release,
	.fb_check_var = msmfb_check_var,
	.fb_set_par = msmfb_set_par,
	.fb_pan_display = msmfb_pan_display,
	.fb_fillrect = msmfb_fillrect,
	.fb_copyarea = msmfb_copyarea,
	.fb_imageblit = msmfb_imageblit,
	.fb_ioctl = msmfb_ioctl,
#ifdef CONFIG_MSMFB_FBIOBLANK
	.fb_blank = msmfb_blank,
#endif
};

static unsigned PP[16];


static void setup_fb_info(struct msmfb_info *msmfb)
{
	struct fb_info *fb_info = msmfb->fb;
	int r;

	/* finish setting up the fb_info struct */
	strncpy(fb_info->fix.id, "msmfb", 16);
	fb_info->fix.ypanstep = 1;

	fb_info->fbops = &msmfb_ops;
	fb_info->flags = FBINFO_DEFAULT;

	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR;
	fb_info->fix.line_length = msmfb->xres * 2;
	fb_info->var.xres = msmfb->xres;
	fb_info->var.yres = msmfb->yres;
	fb_info->var.width = msmfb->panel->fb_data->width;
	fb_info->var.height = msmfb->panel->fb_data->height;
	fb_info->var.xres_virtual = msmfb->xres;
	fb_info->var.yres_virtual = msmfb->yres * 2;
	fb_info->var.bits_per_pixel = 16;
	fb_info->var.accel_flags = 0;

	fb_info->var.yoffset = 0;

	if (msmfb->panel->caps & MSMFB_CAP_PARTIAL_UPDATES) {
		/* set the param in the fixed screen, so userspace can't
		 * change it. This will be used to check for the
		 * capability. */
		fb_info->fix.reserved[0] = 0x5444;
		fb_info->fix.reserved[1] = 0x5055;

		/* This preloads the value so that if userspace doesn't
		 * change it, it will be a full update */
		fb_info->var.reserved[0] = 0x54445055;
		fb_info->var.reserved[1] = 0;
		fb_info->var.reserved[2] = (uint16_t)msmfb->xres |
					   ((uint32_t)msmfb->yres << 16);
	}

	fb_info->var.red.offset = 11;
	fb_info->var.red.length = 5;
	fb_info->var.red.msb_right = 0;
	fb_info->var.green.offset = 5;
	fb_info->var.green.length = 6;
	fb_info->var.green.msb_right = 0;
	fb_info->var.blue.offset = 0;
	fb_info->var.blue.length = 5;
	fb_info->var.blue.msb_right = 0;

	mdp->set_output_format(mdp, fb_info->var.bits_per_pixel);

	r = fb_alloc_cmap(&fb_info->cmap, 16, 0);
	fb_info->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++)
		PP[r] = 0xffffffff;
}

static int setup_fbmem(struct msmfb_info *msmfb, struct platform_device *pdev)
{
	struct fb_info *fb = msmfb->fb;
	struct resource *resource;
	unsigned long size = msmfb->xres * msmfb->yres *
		BYTES_PER_PIXEL(msmfb) * 2;
	unsigned long resource_size;
	unsigned char *fbram;

	/* board file might have attached a resource describing an fb */
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource)
		return -EINVAL;
	resource_size = resource->end - resource->start + 1;

	/* check the resource is large enough to fit the fb */
	if (resource_size < size) {
		printk(KERN_ERR "msmfb: allocated resource is too small for "
				"fb\n");
		return -ENOMEM;
	}
	fb->fix.smem_start = resource->start;
	fb->fix.smem_len = resource_size;
	fbram = ioremap(resource->start, resource_size);
	if (fbram == 0) {
		printk(KERN_ERR "msmfb: cannot allocate fbram!\n");
		return -ENOMEM;
	}

	fb->screen_base = fbram;
	return 0;
}

static int msmfb_probe(struct platform_device *pdev)
{
	struct fb_info *fb;
	struct msmfb_info *msmfb;
	struct msm_panel_data *panel = pdev->dev.platform_data;
	int ret;

	if (!panel) {
		pr_err("msmfb_probe: no platform data\n");
		return -EINVAL;
	}
	if (!panel->fb_data) {
		pr_err("msmfb_probe: no fb_data\n");
		return -EINVAL;
	}

	fb = framebuffer_alloc(sizeof(struct msmfb_info), &pdev->dev);
	if (!fb)
		return -ENOMEM;
	msmfb = fb->par;
	msmfb->fb = fb;
	msmfb->panel = panel;
	msmfb->xres = panel->fb_data->xres;
	msmfb->yres = panel->fb_data->yres;

	ret = setup_fbmem(msmfb, pdev);
	if (ret)
		goto error_setup_fbmem;

	setup_fb_info(msmfb);

	spin_lock_init(&msmfb->update_lock);
	mutex_init(&msmfb->panel_init_lock);
	init_waitqueue_head(&msmfb->frame_wq);
	msmfb->resume_workqueue = create_workqueue("panel_on");
	if (msmfb->resume_workqueue == NULL) {
		printk(KERN_ERR "failed to create panel_on workqueue\n");
		ret = -ENOMEM;
		goto error_create_workqueue;
	}
	INIT_WORK(&msmfb->resume_work, power_on_panel);
	msmfb->black = kzalloc(msmfb->fb->var.bits_per_pixel*msmfb->xres,
			       GFP_KERNEL);

	wake_lock_init(&msmfb->idle_lock, WAKE_LOCK_IDLE, "msmfb_idle_lock");

#ifdef CONFIG_HAS_EARLYSUSPEND
	msmfb->early_suspend.suspend = msmfb_suspend;
	msmfb->early_suspend.resume = msmfb_resume;
	msmfb->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&msmfb->early_suspend);

	msmfb->earlier_suspend.suspend = msmfb_earlier_suspend;
	msmfb->earlier_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&msmfb->earlier_suspend);
#endif

	printk(KERN_INFO "msmfb_probe() installing %d x %d panel\n",
	       msmfb->xres, msmfb->yres);

	msmfb->dma_callback.func = msmfb_handle_dma_interrupt;
	msmfb->vsync_callback.func = msmfb_handle_vsync_interrupt;
	hrtimer_init(&msmfb->fake_vsync, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);


	msmfb->fake_vsync.function = msmfb_fake_vsync;

	ret = register_framebuffer(fb);
	if (ret)
		goto error_register_framebuffer;

	msmfb->sleeping = WAKING;

	return 0;

error_register_framebuffer:
	wake_lock_destroy(&msmfb->idle_lock);
	destroy_workqueue(msmfb->resume_workqueue);
error_create_workqueue:
	iounmap(fb->screen_base);
error_setup_fbmem:
	framebuffer_release(msmfb->fb);
	return ret;
}

static struct platform_driver msm_panel_driver = {
	/* need to write remove */
	.probe = msmfb_probe,
	.driver = {.name = "msm_panel"},
};


static int msmfb_add_mdp_device(struct device *dev,
				struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (mdp)
		return 0;
	mdp = container_of(dev, struct mdp_device, dev);
	return platform_driver_register(&msm_panel_driver);
}

static void msmfb_remove_mdp_device(struct device *dev,
				struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (dev != &mdp->dev)
		return;
	platform_driver_unregister(&msm_panel_driver);
	mdp = NULL;
}

static struct class_interface msm_fb_interface = {
	.add_dev = &msmfb_add_mdp_device,
	.remove_dev = &msmfb_remove_mdp_device,
};

static int __init msmfb_init(void)
{
	return register_mdp_client(&msm_fb_interface);
}

module_init(msmfb_init);
