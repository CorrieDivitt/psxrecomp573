#include "xr_origin_diagnostics.h"
/* Opt-in Win32 (and, psxrecomp573, Android/Quest) OpenXR lifecycle. No guest state, console diagnostics or timing
 * phases: located poses and projection submission belong to one host frame.
 *
 * THREADING: every entry point runs on the emulation/main thread that owns the
 * GL context. Plugin hooks (psx_mod_openxr_*), the GL present path
 * (gl_swap_with_osd) and TCP handlers (executed at the debug_server_poll safe
 * point; the debug I/O thread only queues requests) are all that thread, and
 * no other thread creates a GL context or touches XR. Hence no locking. */
#include "psx_openxr.h"
#include "vr_pose_math.h"
#include "openxr_color.h"
#include "frame_trace.h"
#include "gpu_gl_renderer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
static PSXOpenXRStats s_stats;
static PSXModOpenXRSurface s_surfaces[2];
static uint8_t *s_surface_pixels[2];
static size_t s_surface_capacity[2];
static unsigned s_surface_mask;

static uint64_t s_pair_id,s_pair_cycle;
static uint32_t s_source;
static uint64_t s_native_frame;
static double s_quad_distance,s_quad_width,s_quad_height;
static PSXModOpenXRInput s_input;
static PSXModOpenXRHands s_hands;
static PSXModOpenXRHandJoints s_joints;   /* hand tracking, located with s_hands */
/* Meta's hand mesh per hand (XR_FB_hand_tracking_mesh), fetched once the
 * trackers exist; never freed while running, so pointers handed out stay good. */
typedef struct {
    uint32_t generation,jc,vc,ic;
    float *bind_p,*bind_q,*pos,*nrm,*uv,*bw;
    int16_t *bi,*idx;
} psx_hand_mesh;
static psx_hand_mesh s_hmesh[2];
static uint32_t s_hmesh_gen;
static uint64_t s_hands_sequence, s_hands_sample_ms;
static void hands_clear(void) {
    memset(&s_hands,0,sizeof s_hands);s_hands.struct_size=sizeof s_hands;
    s_hands.sequence=++s_hands_sequence;s_hands.age_ms=UINT32_MAX;
    s_hands_sample_ms=0;
}
static int s_input_override;
static PSXModOpenXRInput s_injected;
static int s_hands_override;
static PSXModOpenXRHands s_injected_hands;
static int valid_pose_numbers(const double q[4],const double p[3]) {
    double n=0;
    for(int i=0;i<4;i++){if(!isfinite(q[i]))return 0;n+=q[i]*q[i];}
    for(int i=0;i<3;i++)if(!isfinite(p[i]) || fabs(p[i])>100)return 0;
    return n>=.5 && n<=1.5;
}
/* Test-only synthetic head motion: translation in room-space metres and an optional pitch, added to
 * both eyes after the origin is established so it reads as the head moving in the world. The
 * translation checks that the world shifts rigidly with the head. The pitch exists because the
 * defects that only appear when looking down cannot be reached by translation, and because the only
 * other look path is unwired. Armed by the first override call; default off, so shipped behaviour
 * is unchanged. */
static int s_pose_bob_armed;
static double s_pose_bob_m[3];
static double s_pose_bob_pitch_deg;
void psx_openxr_pose_bob(double q[2][4],double p[2][3]) {
    if(!s_pose_bob_armed)return;
    for(int i=0;i<2;i++)for(int j=0;j<3;j++)p[i][j]+=s_pose_bob_m[j];
    if(s_pose_bob_pitch_deg!=0.0) {
        double h=s_pose_bob_pitch_deg*(3.141592653589793/180.0)*0.5,sq=sin(h),cq=cos(h);
        for(int i=0;i<2;i++) {
            double x=q[i][0],y=q[i][1],z=q[i][2],w=q[i][3];
            /* Left multiply by a rotation about the XR X axis: a look up or down. */
            q[i][0]=sq*w+cq*x;q[i][1]=cq*y-sq*z;q[i][2]=sq*y+cq*z;q[i][3]=cq*w-sq*x;
        }
    }
}
int psx_openxr_pose_override(double x_m,double y_m,double z_m) {
    if(!isfinite(x_m)||!isfinite(y_m)||!isfinite(z_m))return 0;
    if(fabs(x_m)>10.0||fabs(y_m)>10.0||fabs(z_m)>10.0)return 0;
    s_pose_bob_m[0]=x_m;s_pose_bob_m[1]=y_m;s_pose_bob_m[2]=z_m;s_pose_bob_armed=1;
    return 1;
}
int psx_openxr_pose_pitch_override(double pitch_deg) {
    if(!isfinite(pitch_deg) || fabs(pitch_deg)>90.0)return 0;
    s_pose_bob_pitch_deg=pitch_deg;s_pose_bob_armed=1;
    return 1;
}
void psx_openxr_pose_clear(void) {
    s_pose_bob_m[0]=s_pose_bob_m[1]=s_pose_bob_m[2]=0.0;s_pose_bob_pitch_deg=0.0;s_pose_bob_armed=0;
}
int psx_openxr_hands_override(const PSXModOpenXRHands *hands) {
    if(!hands){s_hands_override=0;hands_clear();return 1;}
    if(hands->struct_size!=sizeof *hands || hands->focused>1 || hands->origin_valid>1)return 0;
    for(int i=0;i<4;i++)if(!isfinite(hands->origin_orientation_xyzw[i]))return 0;
    for(int i=0;i<3;i++)if(!isfinite(hands->origin_position_m[i]) || fabs(hands->origin_position_m[i])>100)return 0;
    if(hands->origin_valid && !valid_pose_numbers(hands->origin_orientation_xyzw,hands->origin_position_m))return 0;
    for(int e=0;e<2;e++)for(int k=0;k<2;k++) {
        const PSXModTrackedPose *v=&hands->pose[e][k];
        if(v->active>1 || (v->flags & ~15u))return 0;
        double q[4],p[3];for(int i=0;i<4;i++)q[i]=v->orientation_xyzw[i];
        for(int i=0;i<3;i++)p[i]=v->position_m[i];
        for(int i=0;i<4;i++)if(!isfinite(q[i]))return 0;
        for(int i=0;i<3;i++)if(!isfinite(p[i]) || fabs(p[i])>100)return 0;
        if((v->flags & 3u) && !valid_pose_numbers(q,p))return 0;
    }
    s_injected_hands=*hands;s_injected_hands.synthetic=1;
    s_injected_hands.sequence=++s_hands_sequence;
    for(int e=0;e<2;e++)for(int k=0;k<2;k++)
        if(!hands->focused || !hands->origin_valid || !hands->pose[e][k].active)
            memset(&s_injected_hands.pose[e][k],0,sizeof(PSXModTrackedPose));
    s_hands_override=1;return 1;
}
int psx_openxr_input_override(const PSXModOpenXRInput *input) {
    if (!input) { s_input_override=0; memset(&s_injected,0,sizeof s_injected); return 1; }
    if (input->struct_size!=sizeof *input || input->focused>1 ||
        input->active[0]>1 || input->active[1]>1) return 0;
    for(int e=0;e<2;e++) for(int a=0;a<2;a++)
        if(!isfinite(input->stick[e][a]) || fabsf(input->stick[e][a])>1) return 0;
    for(int e=0;e<2;e++) {
        if(input->trigger_active[e]>1 || input->squeeze_active[e]>1 ||
           (input->buttons[e] & ~PSX_MOD_XR_CLICKS) ||
           (input->buttons_active[e] & ~PSX_MOD_XR_CLICKS) ||
           !isfinite(input->trigger[e]) || input->trigger[e]<0 || input->trigger[e]>1 ||
           !isfinite(input->squeeze[e]) || input->squeeze[e]<0 || input->squeeze[e]>1)return 0;
    }
    s_injected=*input; s_input_override=1; return 1;
}
void psx_openxr_input_snapshot(PSXModOpenXRInput *out) { if(out)*out=s_input; }
void psx_openxr_pair_metadata(uint64_t id,uint64_t cycle){s_pair_id=id;s_pair_cycle=cycle;s_source=1;s_native_frame=0;}
void psx_openxr_native_metadata(uint64_t frame){s_pair_id=s_pair_cycle=0;s_source=2;s_native_frame=frame;}
int psx_openxr_surface_valid(const PSXModOpenXRSurface *v) {
    if(!v || v->struct_size!=sizeof *v || v->id>=2 || !v->rgba ||
       !v->width_px || !v->height_px || v->width_px>1024 || v->height_px>1024)return 0;
    if(v->flags&~PSX_MOD_OPENXR_SURFACE_FACE_HEAD)return 0;
    double norm=0;
    for(int i=0;i<3;i++)if(!isfinite(v->position_m[i]) || fabs(v->position_m[i])>100)return 0;
    for(int i=0;i<4;i++){if(!isfinite(v->orientation_xyzw[i]))return 0;norm+=v->orientation_xyzw[i]*v->orientation_xyzw[i];}
    if(fabs(norm-1)>.01)return 0;
    for(int i=0;i<2;i++)if(!isfinite(v->size_m[i]) || v->size_m[i]<.001f || v->size_m[i]>2)return 0;
    return 1;
}
#if defined(PSX_OPENXR)
#include "host_time.h"
#if defined(__ANDROID__)
/* Android / Meta Quest (psxrecomp573, 2026-11-23): OpenGL ES 3.2 through EGL,
 * the Khronos loader initialised with the app's Java VM and activity (from
 * SDL). Everything after session creation is the same as on Win32. */
#include <EGL/egl.h>
#include <GLES3/gl32.h>
#include <jni.h>
#include <SDL3/SDL_system.h>
#define XR_USE_PLATFORM_ANDROID
#define XR_USE_GRAPHICS_API_OPENGL_ES
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
typedef XrSwapchainImageOpenGLESKHR psx_xr_image_t;
#define PSX_XR_IMAGE_TYPE XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR
#define PSX_XR_GL_EXTENSION XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <unknwn.h>
#include <GL/gl.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_OPENGL
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
typedef XrSwapchainImageOpenGLKHR psx_xr_image_t;
#define PSX_XR_IMAGE_TYPE XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR
#define PSX_XR_GL_EXTENSION XR_KHR_OPENGL_ENABLE_EXTENSION_NAME
#endif

/* Frame-trace stage helper: perf ticks -> microseconds (CPU submission/wait
 * wall time only, never GPU execution time). */
static uint64_t xr_trace_us(uint64_t ticks) {
    static double inv = 0.0;
    if (inv == 0.0) {
        double f = (double)gl_renderer_perf_frequency();
        inv = f > 0.0 ? 1000000.0 / f : 1.0;
    }
    return (uint64_t)((double)ticks * inv);
}
static XrInstance s_instance;
#if defined(__ANDROID__)
static int s_fov_on,s_fov_level;
static int s_ht_on;                       /* XR_EXT_hand_tracking enabled */
static int s_hm_on;                       /* XR_FB_hand_tracking_mesh enabled */
static int s_aim_on;                      /* XR_FB_hand_tracking_aim enabled (Meta's palm menu pinch) */
static int s_ds_on;                       /* XR_EXT_hand_tracking_data_source: bare hands only */
static int s_sim_on;                      /* XR_META_simultaneous_hands_and_controllers enabled */
static int s_sim_started;
static XrHandTrackerEXT s_ht[2];
static PFN_xrLocateHandJointsEXT s_ht_locate;
static PFN_xrDestroyHandTrackerEXT s_ht_destroy;
#endif
static XrSystemId s_system;
static XrSession s_session;
static XrSpace s_space;
static XrSpace s_view_space;
static XrSpace s_stage_space; /* floor-level STAGE space, if the runtime has one */
static XrActionSet s_actions;
static XrAction s_stick;
static XrAction s_trigger,s_squeeze,s_click[4];
static XrAction s_pose_action[2];
static XrAction s_haptic; /* vibration output, both hands */
static XrSpace s_pose_space[2][2];
static XrPath s_hand[2];
static XrTime s_origin_reset_time;
static unsigned s_origin_inversion_streak;
static XrSwapchain s_chain[2];
static psx_xr_image_t *s_images[2];
static uint32_t s_count[2];
static int s_w[2],s_h[2],s_origin_valid;
static XrView s_views[2];
static PSXModRenderView s_render[2];
static double s_oq[4],s_op[3];
static XrTime s_time;
/* Cadence telemetry for the frame trace: the runtime's own predicted display
 * time delta between submissions (what the compositor actually saw), and when
 * the pose for the current frame was sampled (xrLocateViews return). */
static XrTime s_prev_display_time;
static int s_prev_display_valid;
static uint32_t s_display_delta_us;
static uint64_t s_pose_sample_ticks;
/* Loss handling. A lost session/instance (or an unusable swapchain) is only
 * flagged where it is detected; service_loss() tears the stack down at the end
 * of the public call so no caller loses handles mid-sequence. */
#define XR_RETRY_MS 3000u
static int s_reset_pending,s_exit_pending,s_recovering;
static const char *s_reset_stage;
static uint64_t s_retry_ms;
static int check(XrResult r,const char *stage) {
    s_stats.result=r;
    if(XR_FAILED(r)) {
        s_stats.stage=s_stats.last_failure=stage;s_stats.last_failure_result=r;s_stats.failures++;
#if defined(__ANDROID__)
        fprintf(stdout,"psxrecomp: OpenXR %s failed (XrResult %d)\n",stage,(int)r);
#endif
        if(r==XR_ERROR_SESSION_LOST || r==XR_ERROR_INSTANCE_LOST) {
            s_reset_pending=1;s_reset_stage=r==XR_ERROR_SESSION_LOST?"session_lost":"instance_lost";
        }
        return 0;
    }
    return 1;
}
static int input_initialize(void) {
    XrActionSetCreateInfo set={XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy(set.actionSetName,"gameplay"); strcpy(set.localizedActionSetName,"Gameplay");
    if(!check(xrCreateActionSet(s_instance,&set,&s_actions),"action_set"))return 0;
    if(!check(xrStringToPath(s_instance,"/user/hand/left",&s_hand[0]),"left_hand_path") ||
       !check(xrStringToPath(s_instance,"/user/hand/right",&s_hand[1]),"right_hand_path"))return 0;
    XrActionCreateInfo action={XR_TYPE_ACTION_CREATE_INFO};
    strcpy(action.actionName,"thumbstick"); strcpy(action.localizedActionName,"Thumbstick");
    action.actionType=XR_ACTION_TYPE_VECTOR2F_INPUT;
    action.countSubactionPaths=2; action.subactionPaths=s_hand;
    if(!check(xrCreateAction(s_actions,&action,&s_stick),"stick_action"))return 0;
    action.actionType=XR_ACTION_TYPE_FLOAT_INPUT;
    strcpy(action.actionName,"trigger");strcpy(action.localizedActionName,"Trigger");
    if(!check(xrCreateAction(s_actions,&action,&s_trigger),"trigger_action"))return 0;
    strcpy(action.actionName,"squeeze");strcpy(action.localizedActionName,"Grip squeeze");
    if(!check(xrCreateAction(s_actions,&action,&s_squeeze),"squeeze_action"))return 0;
    const char *names[4]={"primary","secondary","menu","stick_click"};
    const char *labels[4]={"Primary button","Secondary button","Menu","Stick click"};
    action.actionType=XR_ACTION_TYPE_BOOLEAN_INPUT;
    for(int i=0;i<4;i++) {
        strcpy(action.actionName,names[i]);strcpy(action.localizedActionName,labels[i]);
        if(!check(xrCreateAction(s_actions,&action,&s_click[i]),names[i]))return 0;
    }
    action.actionType=XR_ACTION_TYPE_VIBRATION_OUTPUT;
    strcpy(action.actionName,"haptic");strcpy(action.localizedActionName,"Vibration");
    if(!check(xrCreateAction(s_actions,&action,&s_haptic),"haptic_action"))return 0;
    action.actionType=XR_ACTION_TYPE_POSE_INPUT;
    for(int k=0;k<2;k++) {
        strcpy(action.actionName,k?"aim_pose":"grip_pose");
        strcpy(action.localizedActionName,k?"Aim pose":"Grip pose");
        if(!check(xrCreateAction(s_actions,&action,&s_pose_action[k]),"pose_action"))return 0;
        for(int e=0;e<2;e++) {
            XrActionSpaceCreateInfo ci={XR_TYPE_ACTION_SPACE_CREATE_INFO};
            ci.action=s_pose_action[k];ci.subactionPath=s_hand[e];ci.poseInActionSpace.orientation.w=1;
            if(!check(xrCreateActionSpace(s_session,&ci,&s_pose_space[e][k]),"pose_space"))return 0;
        }
    }
    XrPath profile;
    if(!check(xrStringToPath(s_instance,"/interaction_profiles/oculus/touch_controller",&profile),"touch_profile"))return 0;
    const char *paths[2][7]={
        {"/user/hand/left/input/thumbstick","/user/hand/left/input/trigger/value",
         "/user/hand/left/input/squeeze/value","/user/hand/left/input/x/click",
         "/user/hand/left/input/y/click","/user/hand/left/input/menu/click",
         "/user/hand/left/input/thumbstick/click"},
        {"/user/hand/right/input/thumbstick","/user/hand/right/input/trigger/value",
         "/user/hand/right/input/squeeze/value","/user/hand/right/input/a/click",
         "/user/hand/right/input/b/click",NULL,"/user/hand/right/input/thumbstick/click"}};
    XrAction actions[7]={s_stick,s_trigger,s_squeeze,s_click[0],s_click[1],s_click[2],s_click[3]};
    XrActionSuggestedBinding bindings[19];uint32_t n=0;
    for(int e=0;e<2;e++)for(int a=0;a<7;a++)if(paths[e][a]) {
        bindings[n].action=actions[a];
        if(!check(xrStringToPath(s_instance,paths[e][a],&bindings[n].binding),"touch_input_path"))return 0;
        n++;
    }
    const char *pose_paths[2][2]={{"/user/hand/left/input/grip/pose","/user/hand/left/input/aim/pose"},
                               {"/user/hand/right/input/grip/pose","/user/hand/right/input/aim/pose"}};
    for(int e=0;e<2;e++)for(int k=0;k<2;k++) {
        bindings[n].action=s_pose_action[k];
        if(!check(xrStringToPath(s_instance,pose_paths[e][k],&bindings[n].binding),"pose_binding"))return 0;
        n++;
    }
    for(int e=0;e<2;e++) {
        bindings[n].action=s_haptic;
        if(!check(xrStringToPath(s_instance,e?"/user/hand/right/output/haptic":"/user/hand/left/output/haptic",
                                 &bindings[n].binding),"haptic_binding"))return 0;
        n++;
    }
    XrInteractionProfileSuggestedBinding suggest={XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    suggest.interactionProfile=profile; suggest.countSuggestedBindings=n; suggest.suggestedBindings=bindings;
    s_stats.touch_bindings_result=(int32_t)xrSuggestInteractionProfileBindings(s_instance,&suggest);
    if(!check((XrResult)s_stats.touch_bindings_result,"touch_bindings"))return 0;
    /* Fallback the runtime may bind when it does not offer the Touch profile
     * (or offers it only for other devices): select = primary, menu = menu,
     * plus both poses. Not fatal when refused. */
    {
        XrPath simple;
        XrActionSuggestedBinding sb[10];uint32_t sn=0;
        const char *sp[2][4]={{"/user/hand/left/input/select/click","/user/hand/left/input/menu/click",
                               "/user/hand/left/input/grip/pose","/user/hand/left/input/aim/pose"},
                              {"/user/hand/right/input/select/click","/user/hand/right/input/menu/click",
                               "/user/hand/right/input/grip/pose","/user/hand/right/input/aim/pose"}};
        XrAction sa[4]={s_click[0],s_click[2],s_pose_action[0],s_pose_action[1]};
        int ok=check(xrStringToPath(s_instance,"/interaction_profiles/khr/simple_controller",&simple),"simple_profile");
        for(int e=0;e<2 && ok;e++)for(int a=0;a<4 && ok;a++) {
            sb[sn].action=sa[a];
            ok=check(xrStringToPath(s_instance,sp[e][a],&sb[sn].binding),"simple_input_path");
            sn++;
        }
        for(int e=0;e<2 && ok;e++) {
            sb[sn].action=s_haptic;
            ok=check(xrStringToPath(s_instance,e?"/user/hand/right/output/haptic":"/user/hand/left/output/haptic",
                                    &sb[sn].binding),"simple_haptic_path");
            sn++;
        }
        if(ok) {
            XrInteractionProfileSuggestedBinding ss={XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            ss.interactionProfile=simple;ss.countSuggestedBindings=sn;ss.suggestedBindings=sb;
            s_stats.simple_bindings_result=(int32_t)xrSuggestInteractionProfileBindings(s_instance,&ss);
        }
    }
    XrSessionActionSetsAttachInfo attach={XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    attach.countActionSets=1; attach.actionSets=&s_actions;
    return check(xrAttachSessionActionSets(s_session,&attach),"attach_actions");
}
#if defined(__ANDROID__)
/* The Khronos loader on Android needs the Java VM and the app context before
 * any other call (XR_KHR_loader_init_android). */
static JavaVM *s_jvm;
static jobject s_activity;
static int android_loader_init(void) {
    static int s_done;
    if (s_done) return s_done > 0;
    JNIEnv *env = (JNIEnv *)SDL_GetAndroidJNIEnv();
    s_activity = (jobject)SDL_GetAndroidActivity();
    if (!env || !s_activity || (*env)->GetJavaVM(env, &s_jvm) != 0 || !s_jvm) {
        s_stats.stage = "android_jni"; s_done = -1; return 0;
    }
    PFN_xrInitializeLoaderKHR init = NULL;
    if (xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR", (PFN_xrVoidFunction *)&init) != XR_SUCCESS || !init) {
        s_stats.stage = "loader_init_proc"; s_done = -1; return 0;
    }
    XrLoaderInitInfoAndroidKHR li = { XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR };
    li.applicationVM = s_jvm;
    li.applicationContext = s_activity;
    if (!check(init((const XrLoaderInitInfoBaseHeaderKHR *)&li), "loader_init")) { s_done = -1; return 0; }
    s_done = 1;
    return 1;
}
#endif

static int initialize(void) {
    uint32_t count=0;
    XrExtensionProperties *ext=NULL;
#if defined(__ANDROID__)
    if(!android_loader_init())return 0;
#endif
    if(!check(xrEnumerateInstanceExtensionProperties(NULL,0,&count,NULL),"extensions"))return 0;
    ext=calloc(count,sizeof *ext); if(!ext){s_stats.stage="extensions_memory";return 0;}
    for(uint32_t i=0;i<count;i++)ext[i].type=XR_TYPE_EXTENSION_PROPERTIES;
    int supported=0;
#if defined(__ANDROID__)
    int fov_have=0;   /* Meta fixed foveated rendering: all three extensions */
#endif
    if(check(xrEnumerateInstanceExtensionProperties(NULL,count,&count,ext),"extensions"))
        for(uint32_t i=0;i<count;i++) {
            if(!strcmp(ext[i].extensionName,PSX_XR_GL_EXTENSION))supported=1;
#if defined(__ANDROID__)
            if(!strcmp(ext[i].extensionName,XR_FB_SWAPCHAIN_UPDATE_STATE_EXTENSION_NAME))fov_have|=1;
            if(!strcmp(ext[i].extensionName,XR_FB_FOVEATION_EXTENSION_NAME))fov_have|=2;
            if(!strcmp(ext[i].extensionName,XR_FB_FOVEATION_CONFIGURATION_EXTENSION_NAME))fov_have|=4;
            if(!strcmp(ext[i].extensionName,XR_EXT_HAND_TRACKING_EXTENSION_NAME))fov_have|=8;
            if(!strcmp(ext[i].extensionName,XR_FB_HAND_TRACKING_MESH_EXTENSION_NAME))fov_have|=16;
            if(!strcmp(ext[i].extensionName,XR_FB_HAND_TRACKING_AIM_EXTENSION_NAME))fov_have|=32;
            if(!strcmp(ext[i].extensionName,XR_EXT_HAND_TRACKING_DATA_SOURCE_EXTENSION_NAME))fov_have|=64;
            if(!strcmp(ext[i].extensionName,XR_META_SIMULTANEOUS_HANDS_AND_CONTROLLERS_EXTENSION_NAME))fov_have|=128;
#endif
        }
    free(ext);
    if(!supported){s_stats.stage="opengl_extension_missing";return 0;}
#if defined(__ANDROID__)
    /* PSX_XR_FOVEATION: 0 off, 1 low, 2 medium, 3 high (default). */
    {const char *e=getenv("PSX_XR_FOVEATION");s_fov_level=(e&&*e)?atoi(e):3;
     if(s_fov_level<0)s_fov_level=0;if(s_fov_level>3)s_fov_level=3;}
    s_fov_on=(fov_have&7)==7 && s_fov_level>0;
    /* PSX_XR_HAND_TRACKING=0 leaves hand tracking alone. */
    {const char *e=getenv("PSX_XR_HAND_TRACKING");s_ht_on=(fov_have&8) && !(e&&*e=='0');}
    const char *extensions_all[12];uint32_t extensions_n=0;
    extensions_all[extensions_n++]=PSX_XR_GL_EXTENSION;
    extensions_all[extensions_n++]=XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME;
    if(s_fov_on) {
        extensions_all[extensions_n++]=XR_FB_SWAPCHAIN_UPDATE_STATE_EXTENSION_NAME;
        extensions_all[extensions_n++]=XR_FB_FOVEATION_EXTENSION_NAME;
        extensions_all[extensions_n++]=XR_FB_FOVEATION_CONFIGURATION_EXTENSION_NAME;
    }
    if(s_ht_on)extensions_all[extensions_n++]=XR_EXT_HAND_TRACKING_EXTENSION_NAME;
    s_hm_on=s_ht_on && (fov_have&16);
    if(s_hm_on)extensions_all[extensions_n++]=XR_FB_HAND_TRACKING_MESH_EXTENSION_NAME;
    s_aim_on=s_ht_on && (fov_have&32);
    if(s_aim_on)extensions_all[extensions_n++]=XR_FB_HAND_TRACKING_AIM_EXTENSION_NAME;
    /* PSX_XR_SIMULTANEOUS=0: no hands + controllers at once. The data source
     * (bare hands only) goes with it: a hand holding a controller must not
     * come back as joints made up from the controller. */
    {const char *e=getenv("PSX_XR_SIMULTANEOUS");
     s_sim_on=s_ht_on && (fov_have&128) && !(e&&*e=='0');}
    s_ds_on=s_ht_on && (fov_have&64);
    if(s_ds_on)extensions_all[extensions_n++]=XR_EXT_HAND_TRACKING_DATA_SOURCE_EXTENSION_NAME;
    if(s_sim_on)extensions_all[extensions_n++]=XR_META_SIMULTANEOUS_HANDS_AND_CONTROLLERS_EXTENSION_NAME;
    fprintf(stdout,"psxrecomp: OpenXR hand extensions: tracking %d mesh %d aim %d data-source %d simultaneous %d\n",
            s_ht_on,s_hm_on,s_aim_on,s_ds_on,s_sim_on);
    const char *const *extensions=extensions_all;
    fprintf(stdout,"psxrecomp: OpenXR foveation %s (extensions %s, level %d)\n",s_fov_on?"on":"off",
            (fov_have&7)==7?"present":"missing",s_fov_level);
    XrInstanceCreateInfoAndroidKHR aci={XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
    aci.applicationVM=s_jvm;aci.applicationActivity=s_activity;
#else
    const char *extensions[]={PSX_XR_GL_EXTENSION};
#endif
    XrInstanceCreateInfo ci={XR_TYPE_INSTANCE_CREATE_INFO};
#if defined(__ANDROID__)
    ci.next=&aci;
#endif
    strcpy(ci.applicationInfo.applicationName,"PSXRecomp stereo");
    strcpy(ci.applicationInfo.engineName,"psxrecomp");
    ci.applicationInfo.apiVersion=XR_API_VERSION_1_0;
#if defined(__ANDROID__)
    ci.enabledExtensionCount=extensions_n;ci.enabledExtensionNames=extensions;
#else
    ci.enabledExtensionCount=(uint32_t)(sizeof extensions/sizeof extensions[0]);ci.enabledExtensionNames=extensions;
#endif
    if(!check(xrCreateInstance(&ci,&s_instance),"instance"))return 0;
    XrInstanceProperties ip={XR_TYPE_INSTANCE_PROPERTIES};
    if(check(xrGetInstanceProperties(s_instance,&ip),"instance_properties"))
        memcpy(s_stats.runtime,ip.runtimeName,sizeof s_stats.runtime);
    s_stats.runtime[sizeof s_stats.runtime-1]=0;
    XrSystemGetInfo si={XR_TYPE_SYSTEM_GET_INFO};si.formFactor=XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    if(!check(xrGetSystem(s_instance,&si,&s_system),"system"))return 0;
#if defined(__ANDROID__)
    PFN_xrGetOpenGLESGraphicsRequirementsKHR requirements=NULL;
    if(!check(xrGetInstanceProcAddr(s_instance,"xrGetOpenGLESGraphicsRequirementsKHR",
                                  (PFN_xrVoidFunction*)&requirements),"graphics_requirements_proc"))return 0;
    XrGraphicsRequirementsOpenGLESKHR req={XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
#else
    PFN_xrGetOpenGLGraphicsRequirementsKHR requirements=NULL;
    if(!check(xrGetInstanceProcAddr(s_instance,"xrGetOpenGLGraphicsRequirementsKHR",
                                  (PFN_xrVoidFunction*)&requirements),"graphics_requirements_proc"))return 0;
    XrGraphicsRequirementsOpenGLKHR req={XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
#endif
    if(!requirements || !check(requirements(s_instance,s_system,&req),"graphics_requirements"))return 0;
    GLint major=0,minor=0;glGetIntegerv(0x821B,&major);glGetIntegerv(0x821C,&minor);
    XrVersion version=XR_MAKE_VERSION(major,minor,0);
    s_stats.gl_version=version;s_stats.min_gl_version=req.minApiVersionSupported;
    s_stats.max_gl_version=req.maxApiVersionSupported;
    if(version<req.minApiVersionSupported){s_stats.stage="opengl_version";s_stats.failures++;return 0;}
#if defined(__ANDROID__)
    XrGraphicsBindingOpenGLESAndroidKHR binding={XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
    binding.display=eglGetCurrentDisplay();binding.context=eglGetCurrentContext();
    if(binding.display==EGL_NO_DISPLAY || binding.context==EGL_NO_CONTEXT){s_stats.stage="opengl_context";return 0;}
    {   /* the EGLConfig the current context was made with */
        EGLint cfg_id=0,n=0;
        eglQueryContext(binding.display,binding.context,EGL_CONFIG_ID,&cfg_id);
        const EGLint want[]={EGL_CONFIG_ID,cfg_id,EGL_NONE};
        if(!eglChooseConfig(binding.display,want,&binding.config,1,&n) || n<1){s_stats.stage="egl_config";return 0;}
    }
#else
    XrGraphicsBindingOpenGLWin32KHR binding={XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR};
    binding.hDC=wglGetCurrentDC();binding.hGLRC=wglGetCurrentContext();
    if(!binding.hDC || !binding.hGLRC){s_stats.stage="opengl_context";return 0;}
#endif
    XrSessionCreateInfo sci={XR_TYPE_SESSION_CREATE_INFO};sci.next=&binding;sci.systemId=s_system;
    if(!check(xrCreateSession(s_instance,&sci,&s_session),"session"))return 0;
#if defined(__ANDROID__)
    memset(&s_joints,0,sizeof s_joints);
    if(s_ht_on) {   /* a tracker per hand; a failure only means no hand tracking */
        XrSystemHandTrackingPropertiesEXT hp={XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
        XrSystemProperties sp={XR_TYPE_SYSTEM_PROPERTIES};sp.next=&hp;
        PFN_xrCreateHandTrackerEXT create=NULL;
        int ok=XR_SUCCEEDED(xrGetSystemProperties(s_instance,s_system,&sp)) && hp.supportsHandTracking;
        if(ok) {
            xrGetInstanceProcAddr(s_instance,"xrCreateHandTrackerEXT",(PFN_xrVoidFunction*)&create);
            xrGetInstanceProcAddr(s_instance,"xrLocateHandJointsEXT",(PFN_xrVoidFunction*)&s_ht_locate);
            xrGetInstanceProcAddr(s_instance,"xrDestroyHandTrackerEXT",(PFN_xrVoidFunction*)&s_ht_destroy);
            ok=create && s_ht_locate;
        }
        for(int h=0;ok && h<2;h++) {
            XrHandTrackerCreateInfoEXT hc={XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
            hc.hand=h?XR_HAND_RIGHT_EXT:XR_HAND_LEFT_EXT;hc.handJointSet=XR_HAND_JOINT_SET_DEFAULT_EXT;
            XrHandTrackingDataSourceEXT bare_only[1]={XR_HAND_TRACKING_DATA_SOURCE_UNOBSTRUCTED_EXT};
            XrHandTrackingDataSourceInfoEXT dsi={XR_TYPE_HAND_TRACKING_DATA_SOURCE_INFO_EXT};
            dsi.requestedDataSourceCount=1;dsi.requestedDataSources=bare_only;
            if(s_ds_on)hc.next=&dsi;
            XrResult r=create(s_session,&hc,&s_ht[h]);
            if(XR_FAILED(r)){fprintf(stdout,"psxrecomp: OpenXR hand tracker %d failed (%d)\n",h,(int)r);s_ht[h]=XR_NULL_HANDLE;ok=0;}
        }
        s_joints.supported=(uint32_t)ok;
        s_joints.aim_supported=(uint32_t)(ok && s_aim_on);
        if(s_sim_on) {
            XrSystemSimultaneousHandsAndControllersPropertiesMETA shp={XR_TYPE_SYSTEM_SIMULTANEOUS_HANDS_AND_CONTROLLERS_PROPERTIES_META};
            XrSystemProperties sp2={XR_TYPE_SYSTEM_PROPERTIES};sp2.next=&shp;
            if(!XR_SUCCEEDED(xrGetSystemProperties(s_instance,s_system,&sp2)) || !shp.supportsSimultaneousHandsAndControllers) {
                fprintf(stdout,"psxrecomp: OpenXR simultaneous hands and controllers: not supported by this system\n");
                s_sim_on=0;
            }
        }
        s_joints.scale[0]=s_joints.scale[1]=1.0f;
        /* Meta's hand mesh: ask for the sizes, then the data. */
        PFN_xrGetHandMeshFB get_mesh=NULL;
        if(ok && s_hm_on)xrGetInstanceProcAddr(s_instance,"xrGetHandMeshFB",(PFN_xrVoidFunction*)&get_mesh);
        for(int h=0;get_mesh && h<2;h++) {
            XrHandTrackingMeshFB hm={XR_TYPE_HAND_TRACKING_MESH_FB};
            XrResult r=get_mesh(s_ht[h],&hm);
            if(XR_FAILED(r) || !hm.jointCountOutput || !hm.vertexCountOutput || !hm.indexCountOutput) {
                fprintf(stdout,"psxrecomp: OpenXR hand mesh %d not available (%d)\n",h,(int)r);continue;
            }
            const uint32_t jc=hm.jointCountOutput,vc=hm.vertexCountOutput,ic=hm.indexCountOutput;
            XrPosef *bp=calloc(jc,sizeof *bp);float *rad=calloc(jc,sizeof *rad);XrHandJointEXT *par=calloc(jc,sizeof *par);
            XrVector3f *vp=calloc(vc,sizeof *vp),*vn=calloc(vc,sizeof *vn);XrVector2f *vu=calloc(vc,sizeof *vu);
            XrVector4sFB *vbi=calloc(vc,sizeof *vbi);XrVector4f *vbw=calloc(vc,sizeof *vbw);int16_t *ix=calloc(ic,sizeof *ix);
            psx_hand_mesh m;memset(&m,0,sizeof m);
            m.bind_p=calloc(jc*3,sizeof(float));m.bind_q=calloc(jc*4,sizeof(float));
            m.pos=calloc(vc*3,sizeof(float));m.nrm=calloc(vc*3,sizeof(float));m.uv=calloc(vc*2,sizeof(float));
            m.bw=calloc(vc*4,sizeof(float));m.bi=calloc(vc*4,sizeof(int16_t));m.idx=ix;
            if(!bp||!rad||!par||!vp||!vn||!vu||!vbi||!vbw||!ix||!m.bind_p||!m.bind_q||!m.pos||!m.nrm||!m.uv||!m.bw||!m.bi) {
                fprintf(stdout,"psxrecomp: OpenXR hand mesh %d: out of memory\n",h);continue;
            }
            hm.jointCapacityInput=jc;hm.jointBindPoses=bp;hm.jointRadii=rad;hm.jointParents=par;
            hm.vertexCapacityInput=vc;hm.vertexPositions=vp;hm.vertexNormals=vn;hm.vertexUVs=vu;
            hm.vertexBlendIndices=vbi;hm.vertexBlendWeights=vbw;hm.indexCapacityInput=ic;hm.indices=ix;
            r=get_mesh(s_ht[h],&hm);
            if(XR_FAILED(r)){fprintf(stdout,"psxrecomp: OpenXR hand mesh %d failed (%d)\n",h,(int)r);continue;}
            for(uint32_t j=0;j<jc;j++) {
                m.bind_p[j*3]=bp[j].position.x;m.bind_p[j*3+1]=bp[j].position.y;m.bind_p[j*3+2]=bp[j].position.z;
                m.bind_q[j*4]=bp[j].orientation.x;m.bind_q[j*4+1]=bp[j].orientation.y;
                m.bind_q[j*4+2]=bp[j].orientation.z;m.bind_q[j*4+3]=bp[j].orientation.w;
            }
            for(uint32_t v=0;v<vc;v++) {
                m.pos[v*3]=vp[v].x;m.pos[v*3+1]=vp[v].y;m.pos[v*3+2]=vp[v].z;
                m.nrm[v*3]=vn[v].x;m.nrm[v*3+1]=vn[v].y;m.nrm[v*3+2]=vn[v].z;
                m.uv[v*2]=vu[v].x;m.uv[v*2+1]=vu[v].y;
                m.bi[v*4]=vbi[v].x;m.bi[v*4+1]=vbi[v].y;m.bi[v*4+2]=vbi[v].z;m.bi[v*4+3]=vbi[v].w;
                m.bw[v*4]=vbw[v].x;m.bw[v*4+1]=vbw[v].y;m.bw[v*4+2]=vbw[v].z;m.bw[v*4+3]=vbw[v].w;
            }
            free(bp);free(rad);free(par);free(vp);free(vn);free(vu);free(vbi);free(vbw);
            m.jc=jc;m.vc=vc;m.ic=ic;m.generation=++s_hmesh_gen;
            s_hmesh[h]=m;   /* the old arrays, if any, are left alone: a caller may still hold them */
            fprintf(stdout,"psxrecomp: OpenXR hand mesh %s ready: %u vertices, %u triangles, %u joints\n",
                    h?"right":"left",vc,ic/3,jc);
        }
        fprintf(stdout,"psxrecomp: OpenXR hand tracking %s\n",ok?"ready":"not available (off in the headset's settings?)");
    } else fprintf(stdout,"psxrecomp: OpenXR hand tracking off (extension %s)\n",(fov_have&8)?"present":"missing");
#endif
    if(++s_stats.session_generation==0)s_stats.session_generation=1;
    if(!input_initialize())return 0;
    XrReferenceSpaceCreateInfo space={XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    space.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_LOCAL;space.poseInReferenceSpace.orientation.w=1;
    if(!check(xrCreateReferenceSpace(s_session,&space,&s_space),"local_space"))return 0;
    /* Optional: STAGE has its origin on the real floor (psx_openxr_floor). A
     * runtime without it, or with no floor set up, just leaves the floor unknown. */
    space.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_STAGE;
    if(xrCreateReferenceSpace(s_session,&space,&s_stage_space)!=XR_SUCCESS)s_stage_space=XR_NULL_HANDLE;
    XrViewConfigurationView config[2]={{XR_TYPE_VIEW_CONFIGURATION_VIEW},{XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    if(!check(xrEnumerateViewConfigurationViews(s_instance,s_system,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
              2,&count,config),"view_config") || count!=2)return 0;
    uint32_t formats_n=0;
    if(!check(xrEnumerateSwapchainFormats(s_session,0,&formats_n,NULL),"formats"))return 0;
    int64_t *formats=calloc(formats_n,sizeof *formats);if(!formats){s_stats.stage="formats_memory";return 0;}
    if(!check(xrEnumerateSwapchainFormats(s_session,formats_n,&formats_n,formats),"formats")) {
        free(formats);return 0;
    }
    s_stats.swapchain_format=psx_xr_color_format(formats,formats_n);
    free(formats);
    if(!s_stats.swapchain_format){s_stats.stage="color_format_missing";return 0;}
#if defined(__ANDROID__)
    /* PSX_XR_EYE_SCALE (0.5..1.5): the eye images' size against the runtime's
     * recommended size (Quest 3: 1680x1760). */
    double eye_scale=1.0;
    {const char *e=getenv("PSX_XR_EYE_SCALE");if(e&&*e){eye_scale=atof(e);
     if(eye_scale<0.5)eye_scale=0.5;if(eye_scale>1.5)eye_scale=1.5;}}
    PFN_xrCreateFoveationProfileFB create_profile=NULL;PFN_xrDestroyFoveationProfileFB destroy_profile=NULL;
    PFN_xrUpdateSwapchainFB update_swapchain=NULL;
    if(s_fov_on) {
        xrGetInstanceProcAddr(s_instance,"xrCreateFoveationProfileFB",(PFN_xrVoidFunction*)&create_profile);
        xrGetInstanceProcAddr(s_instance,"xrDestroyFoveationProfileFB",(PFN_xrVoidFunction*)&destroy_profile);
        xrGetInstanceProcAddr(s_instance,"xrUpdateSwapchainFB",(PFN_xrVoidFunction*)&update_swapchain);
        if(!create_profile||!update_swapchain)s_fov_on=0;
    }
    XrFoveationProfileFB profile=XR_NULL_HANDLE;
#endif
    for(int eye=0;eye<2;eye++) {
        s_w[eye]=config[eye].recommendedImageRectWidth;s_h[eye]=config[eye].recommendedImageRectHeight;
#if defined(__ANDROID__)
        if(eye_scale!=1.0){
            s_w[eye]=((int)(s_w[eye]*eye_scale)+7)&~7;s_h[eye]=((int)(s_h[eye]*eye_scale)+7)&~7;
            if(s_w[eye]>(int)config[eye].maxImageRectWidth)s_w[eye]=(int)config[eye].maxImageRectWidth;
            if(s_h[eye]>(int)config[eye].maxImageRectHeight)s_h[eye]=(int)config[eye].maxImageRectHeight;
        }
#endif
        XrSwapchainCreateInfo sc={XR_TYPE_SWAPCHAIN_CREATE_INFO};
        sc.usageFlags=XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;sc.format=s_stats.swapchain_format;
        sc.sampleCount=1;sc.width=s_w[eye];sc.height=s_h[eye];sc.faceCount=1;sc.arraySize=1;sc.mipCount=1;
#if defined(__ANDROID__)
        /* GLES: foveation through binning (QCOM_texture_foveated) */
        XrSwapchainCreateInfoFoveationFB fsc={XR_TYPE_SWAPCHAIN_CREATE_INFO_FOVEATION_FB};
        fsc.flags=XR_SWAPCHAIN_CREATE_FOVEATION_SCALED_BIN_BIT_FB;
        if(s_fov_on)sc.next=&fsc;
#endif
        if(!check(xrCreateSwapchain(s_session,&sc,&s_chain[eye]),"swapchain"))return 0;
#if defined(__ANDROID__)
        if(s_fov_on) {
            if(profile==XR_NULL_HANDLE) {
                XrFoveationLevelProfileCreateInfoFB lp={XR_TYPE_FOVEATION_LEVEL_PROFILE_CREATE_INFO_FB};
                lp.level=(XrFoveationLevelFB)s_fov_level;lp.verticalOffset=0.0f;lp.dynamic=XR_FOVEATION_DYNAMIC_DISABLED_FB;
                XrFoveationProfileCreateInfoFB pc={XR_TYPE_FOVEATION_PROFILE_CREATE_INFO_FB};pc.next=&lp;
                XrResult r=create_profile(s_session,&pc,&profile);
                if(XR_FAILED(r)){fprintf(stdout,"psxrecomp: OpenXR foveation profile failed (%d)\n",(int)r);profile=XR_NULL_HANDLE;s_fov_on=0;}
            }
            if(profile!=XR_NULL_HANDLE) {
                XrSwapchainStateFoveationFB st={XR_TYPE_SWAPCHAIN_STATE_FOVEATION_FB};st.profile=profile;
                XrResult r=update_swapchain(s_chain[eye],(const XrSwapchainStateBaseHeaderFB*)&st);
                if(XR_FAILED(r))fprintf(stdout,"psxrecomp: OpenXR foveation on eye %d failed (%d)\n",eye,(int)r);
                else if(eye==1)fprintf(stdout,"psxrecomp: OpenXR foveation applied (level %d)\n",s_fov_level);
            }
        }
#endif
        if(!check(xrEnumerateSwapchainImages(s_chain[eye],0,&s_count[eye],NULL),"images"))return 0;
        s_images[eye]=calloc(s_count[eye],sizeof *s_images[eye]);
        if(!s_images[eye]){s_stats.stage="images_memory";return 0;}
        for(uint32_t i=0;i<s_count[eye];i++)s_images[eye][i].type=PSX_XR_IMAGE_TYPE;
        if(!check(xrEnumerateSwapchainImages(s_chain[eye],s_count[eye],&s_count[eye],
                    (XrSwapchainImageBaseHeader*)s_images[eye]),"images"))return 0;
    }
#if defined(__ANDROID__)
    if(profile!=XR_NULL_HANDLE && destroy_profile)destroy_profile(profile);   /* the swapchains keep their state */
#endif
    s_stats.initialized=1;s_stats.stage="waiting_session";
#if defined(__ANDROID__)
    fprintf(stdout,"psxrecomp: OpenXR session created (%s), eyes %dx%d, swapchain format 0x%llX\n",
            s_stats.runtime,s_w[0],s_h[0],(unsigned long long)s_stats.swapchain_format);
#endif
    return 1;
}
/* Ask for the bound interaction profiles only when the runtime says they
 * changed (plus a few retries at the start while nothing is bound yet), not
 * on a timer: on Virtual Desktop's runtime a once-a-second
 * xrGetCurrentInteractionProfile lined up with headset stutters of 25-55 ms
 * (Fighting Mania VR play log, 2026-11-21). */
static int s_profile_dirty=1, s_profile_retries=10;
static int events(void) {
    for(;;) {
        XrEventDataBuffer event={XR_TYPE_EVENT_DATA_BUFFER};
        XrResult r=xrPollEvent(s_instance,&event);
        if(r==XR_EVENT_UNAVAILABLE)break;
        if(!check(r,"events"))return 0;
        if(event.type==XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            XrEventDataSessionStateChanged *e=(void*)&event;
            s_stats.state=e->state;
            if(e->state==XR_SESSION_STATE_READY && !s_stats.running) {
                XrSessionBeginInfo begin={XR_TYPE_SESSION_BEGIN_INFO};
                begin.primaryViewConfigurationType=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                if(!check(xrBeginSession(s_session,&begin),"begin_session"))return 0;
                s_stats.running=1;
#if defined(__ANDROID__)
                if(s_sim_on && !s_sim_started) {   /* hands and controllers at the same time */
                    PFN_xrResumeSimultaneousHandsAndControllersTrackingMETA resume=NULL;
                    xrGetInstanceProcAddr(s_instance,"xrResumeSimultaneousHandsAndControllersTrackingMETA",(PFN_xrVoidFunction*)&resume);
                    XrSimultaneousHandsAndControllersTrackingResumeInfoMETA ri={XR_TYPE_SIMULTANEOUS_HANDS_AND_CONTROLLERS_TRACKING_RESUME_INFO_META};
                    XrResult rr=resume?resume(s_session,&ri):XR_ERROR_FUNCTION_UNSUPPORTED;
                    s_sim_started=XR_SUCCEEDED(rr);
                    s_joints.simultaneous=(uint32_t)s_sim_started;
                    fprintf(stdout,"psxrecomp: OpenXR simultaneous hands and controllers: %s (%d)\n",s_sim_started?"on":"failed",(int)rr);
                }
#endif
            } else if(e->state==XR_SESSION_STATE_STOPPING) {
                if(!check(xrEndSession(s_session),"end_session"))return 0;
                s_stats.running=0;
            } else if(e->state==XR_SESSION_STATE_LOSS_PENDING) {
                s_reset_pending=1;s_reset_stage="session_loss";return 0;
            } else if(e->state==XR_SESSION_STATE_EXITING) {
                s_exit_pending=1;return 0;
            }
        } else if(event.type==XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING) {
            /* Recenter from the next valid located pair. */
            const XrEventDataReferenceSpaceChangePending *e=(const void*)&event;
            if(e->referenceSpaceType==XR_REFERENCE_SPACE_TYPE_LOCAL) s_origin_reset_time=e->changeTime;
        } else if(event.type==XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED) {
            s_profile_dirty=1;
        } else if(event.type==XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
            s_reset_pending=1;s_reset_stage="instance_loss";return 0;
        }
    }
    return 1;
}
/* Sync the gameplay action set. Needed by both the input poll and hand
 * location (a pose action is only active after a sync), so a plugin that only
 * calls psx_mod_openxr_begin still gets hands. True only for XR_SUCCESS: the
 * session is focused and the actions are current. */
static int sync_actions(void) {
    XrActiveActionSet active={s_actions,XR_NULL_PATH};
    XrActionsSyncInfo sync={XR_TYPE_ACTIONS_SYNC_INFO};sync.countActiveActionSets=1;sync.activeActionSets=&active;
    XrResult r=xrSyncActions(s_session,&sync);
    return check(r,"sync_actions") && r==XR_SUCCESS;
}
/* Focus for hands comes from the session state events and a successful action
 * sync in this call, never from whether the plugin polled psx_mod_openxr_input. */
static void hands_locate(int focused) {
    PSXModOpenXRHands current=s_hands;
    current.predicted_time=(uint64_t)s_time;
    current.focused=focused && s_stats.state==XR_SESSION_STATE_FOCUSED;
    current.origin_valid=s_origin_valid;
    memcpy(current.origin_position_m,s_op,sizeof s_op);
    memcpy(current.origin_orientation_xyzw,s_oq,sizeof s_oq);
    if(current.focused && current.origin_valid)for(int e=0;e<2;e++)for(int k=0;k<2;k++) {
        XrActionStateGetInfo get={XR_TYPE_ACTION_STATE_GET_INFO};
        get.action=s_pose_action[k];get.subactionPath=s_hand[e];
        XrActionStatePose state={XR_TYPE_ACTION_STATE_POSE};
        if(!check(xrGetActionStatePose(s_session,&get,&state),"pose_state"))continue;
        PSXModTrackedPose *v=&current.pose[e][k];v->active=state.isActive;
        if(!state.isActive)continue;
        XrSpaceLocation loc={XR_TYPE_SPACE_LOCATION};
        XrResult r=xrLocateSpace(s_pose_space[e][k],s_space,s_time,&loc);
        if(!check(r,"locate_hand") || r!=XR_SUCCESS)continue;
        v->flags=(uint32_t)loc.locationFlags & 15u;
        if((v->flags & 3u)!=3u)continue; /* Values remain zero for incomplete poses. */
        double norm=0;float q[4]={loc.pose.orientation.x,loc.pose.orientation.y,
                               loc.pose.orientation.z,loc.pose.orientation.w};
        float p[3]={loc.pose.position.x,loc.pose.position.y,loc.pose.position.z};
        int valid=1;
        for(int i=0;i<4;i++){if(!isfinite(q[i]))valid=0;norm+=(double)q[i]*q[i];}
        for(int i=0;i<3;i++)if(!isfinite(p[i]) || fabsf(p[i])>100)valid=0;
        if(!valid || norm<.5 || norm>1.5){v->flags=0;continue;}
        memcpy(v->orientation_xyzw,q,sizeof q);memcpy(v->position_m,p,sizeof p);
    }
    s_hands=current;s_hands_sample_ms=psx_host_mono_ms();
#if defined(__ANDROID__)
    if(s_joints.supported)for(int h=0;h<2;h++) {
        s_joints.active[h]=0;s_joints.aim_status[h]=0;s_joints.pinch_index[h]=0;s_joints.velocity_valid[h]=0;
        if(!current.focused || !s_ht[h])continue;
        XrHandJointLocationEXT jl[XR_HAND_JOINT_COUNT_EXT];
        XrHandJointLocationsEXT locs={XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
        locs.jointCount=XR_HAND_JOINT_COUNT_EXT;locs.jointLocations=jl;
        XrHandTrackingScaleFB hs={XR_TYPE_HAND_TRACKING_SCALE_FB};   /* the player's hand size */
        hs.currentOutput=1.0f;
        XrHandTrackingAimStateFB as={XR_TYPE_HAND_TRACKING_AIM_STATE_FB};   /* Meta's pinches and palm menu gesture */
        XrHandJointVelocityEXT jv[XR_HAND_JOINT_COUNT_EXT];
        XrHandJointVelocitiesEXT vels={XR_TYPE_HAND_JOINT_VELOCITIES_EXT};
        vels.jointCount=XR_HAND_JOINT_COUNT_EXT;vels.jointVelocities=jv;
        {   void *chain=NULL;
            vels.next=chain;chain=&vels;
            if(s_aim_on){as.next=chain;chain=&as;}
            if(s_hm_on){hs.next=chain;chain=&hs;}
            locs.next=chain; }
        XrHandJointsLocateInfoEXT li={XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
        li.baseSpace=s_space;li.time=s_time;
        if(XR_FAILED(s_ht_locate(s_ht[h],&li,&locs)) || !locs.isActive)continue;
        int palm_ok=0;
        for(int j=0;j<PSX_MOD_XR_HAND_JOINTS && j<XR_HAND_JOINT_COUNT_EXT;j++) {
            const XrHandJointLocationEXT *l=&jl[j];
            int v=(l->locationFlags&3u)==3u;
            const float q[4]={l->pose.orientation.x,l->pose.orientation.y,l->pose.orientation.z,l->pose.orientation.w};
            const float p[3]={l->pose.position.x,l->pose.position.y,l->pose.position.z};
            double norm=0;
            for(int i=0;i<4;i++){if(!isfinite(q[i]))v=0;norm+=(double)q[i]*q[i];}
            for(int i=0;i<3;i++)if(!isfinite(p[i]) || fabsf(p[i])>100)v=0;
            if(norm<.5 || norm>1.5)v=0;
            s_joints.valid[h][j]=(uint32_t)v;
            if(!v)continue;
            memcpy(s_joints.position_m[h][j],p,sizeof p);memcpy(s_joints.orientation_xyzw[h][j],q,sizeof q);
            s_joints.radius_m[h][j]=isfinite(l->radius)?l->radius:0.01f;
            if(j==0)palm_ok=1;
        }
        s_joints.active[h]=(uint32_t)palm_ok;
        s_joints.aim_status[h]=s_aim_on?(uint32_t)(as.status & 0xFFFFFFFFu):0u;
        s_joints.pinch_index[h]=(s_aim_on && isfinite(as.pinchStrengthIndex))?as.pinchStrengthIndex:0.0f;
        {   const XrHandJointVelocityEXT *pv=&jv[XR_HAND_JOINT_PALM_EXT];
            int vv=(pv->velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT)!=0 &&
                   isfinite(pv->linearVelocity.x) && isfinite(pv->linearVelocity.y) && isfinite(pv->linearVelocity.z);
            s_joints.velocity_valid[h]=(uint32_t)vv;
            if(vv){s_joints.palm_velocity_mps[h][0]=pv->linearVelocity.x;s_joints.palm_velocity_mps[h][1]=pv->linearVelocity.y;
                   s_joints.palm_velocity_mps[h][2]=pv->linearVelocity.z;} }
        s_joints.scale[h]=(s_hm_on && isfinite(hs.currentOutput) && hs.currentOutput>0.5f && hs.currentOutput<2.0f)?hs.currentOutput:1.0f;
    }
#endif
    /* Which interaction profile the runtime bound to each hand (diagnostic,
     * openxr_stats.hand_profile): on the INTERACTION_PROFILE_CHANGED event,
     * and once a second for the first few seconds until both hands have one. */
    static uint64_t profile_ms;
    if(s_profile_dirty || (s_profile_retries>0 && s_hands_sample_ms-profile_ms>=1000u)) {
        const uint64_t t0=psx_host_mono_ms();
        profile_ms=s_hands_sample_ms;
        s_profile_dirty=0;
        if(s_profile_retries>0)s_profile_retries--;
        for(int e=0;e<2;e++) {
            XrInteractionProfileState ps={XR_TYPE_INTERACTION_PROFILE_STATE};
            s_stats.hand_profile[e][0]=0;
            if(xrGetCurrentInteractionProfile(s_session,s_hand[e],&ps)!=XR_SUCCESS ||
               ps.interactionProfile==XR_NULL_PATH)continue;
            uint32_t n=0;
            (void)xrPathToString(s_instance,ps.interactionProfile,(uint32_t)sizeof s_stats.hand_profile[e],
                                 &n,s_stats.hand_profile[e]);
            s_stats.hand_profile[e][sizeof s_stats.hand_profile[e]-1]=0;
        }
        if(s_stats.hand_profile[0][0] && s_stats.hand_profile[1][0])s_profile_retries=0;
        const uint64_t took=psx_host_mono_ms()-t0;
        if(took>=5u)fprintf(stderr,"psxrecomp: OpenXR: reading the controller profiles took %llu ms\n",(unsigned long long)took);
    }
}
/* Destroy every XR object (children before parents). Safe on a partially
 * initialised stack and on a lost session: destroy calls need no live session. */
static void xr_destroy(void) {
    /* A new session must not inherit the previous one's display times: the
     * first delta after a re-init would be meaningless. */
    s_prev_display_valid=0;s_prev_display_time=0;
    s_profile_dirty=1;s_profile_retries=10;   /* a new session binds again */
    for(int eye=0;eye<2;eye++) {
        for(int k=0;k<2;k++) {
            if(s_pose_space[eye][k])xrDestroySpace(s_pose_space[eye][k]);
            s_pose_space[eye][k]=XR_NULL_HANDLE;
        }
        if(s_chain[eye])xrDestroySwapchain(s_chain[eye]);
        s_chain[eye]=XR_NULL_HANDLE;free(s_images[eye]);s_images[eye]=NULL;s_count[eye]=0;
    }
#if defined(__ANDROID__)
    for(int h=0;h<2;h++){if(s_ht[h] && s_ht_destroy)s_ht_destroy(s_ht[h]);s_ht[h]=XR_NULL_HANDLE;}
    s_joints.supported=0;s_joints.active[0]=s_joints.active[1]=0;
#endif
    if(s_space)xrDestroySpace(s_space);s_space=XR_NULL_HANDLE;
    if(s_view_space)xrDestroySpace(s_view_space);s_view_space=XR_NULL_HANDLE;
    if(s_stage_space)xrDestroySpace(s_stage_space);s_stage_space=XR_NULL_HANDLE;
    if(s_session)xrDestroySession(s_session);s_session=XR_NULL_HANDLE;
    if(s_actions)xrDestroyActionSet(s_actions);s_actions=XR_NULL_HANDLE;s_stick=XR_NULL_HANDLE;
    s_trigger=s_squeeze=XR_NULL_HANDLE;memset(s_click,0,sizeof s_click);
    memset(s_pose_action,0,sizeof s_pose_action);s_haptic=XR_NULL_HANDLE;
    if(s_instance)xrDestroyInstance(s_instance);s_instance=XR_NULL_HANDLE;
    s_origin_valid=0;s_origin_reset_time=0;s_origin_inversion_streak=0;
    s_stats.origin_inversion_streak=s_stats.origin_ambiguous=0;s_stats.origin_up_dot=1.0f;
    s_stats.state=0;
    s_stats.initialized=s_stats.running=s_stats.tracking=s_stats.frame_open=0;
}
/* Act on a flagged loss. A lost session/instance or an unusable swapchain tears
 * everything down so the caller falls back to the flat present, then keeps XR
 * enabled and re-initialises after XR_RETRY_MS. EXITING (the user closed the
 * headset app) tears down and leaves XR disabled until an explicit enable.
 * Returns 1 if it tore the stack down. Call only at the end of a public entry. */
static int service_loss(void) {
    if(!s_reset_pending && !s_exit_pending)return 0;
    const int exiting=s_exit_pending && !s_reset_pending;
    xr_destroy(); /* a lost session cannot end its frame; xr_destroy clears frame_open */
    memset(&s_input,0,sizeof s_input);s_input.struct_size=sizeof s_input;
    hands_clear();
    s_quad_distance=s_quad_width=s_quad_height=0;
    if(exiting) {
        s_stats.enabled=0;s_stats.stage="session_exit";s_recovering=0;s_retry_ms=0;
    } else {
        s_stats.losses++;s_stats.stage=s_reset_stage?s_reset_stage:"lost";
        s_recovering=1;s_retry_ms=psx_host_mono_ms()+XR_RETRY_MS;
    }
    s_reset_pending=s_exit_pending=0;s_reset_stage=NULL;
    return 1;
}
#endif
int psx_openxr_hand_mesh(uint32_t hand,PSXModOpenXRHandMesh *out) {
    if(!out || out->struct_size!=sizeof *out || hand>1)return 0;
    const psx_hand_mesh *m=&s_hmesh[hand];
    if(!m->generation || !m->vc || !m->ic)return 0;
    out->generation=m->generation;out->joint_count=m->jc;out->vertex_count=m->vc;out->index_count=m->ic;
    out->bind_position=m->bind_p;out->bind_orientation_xyzw=m->bind_q;
    out->position=m->pos;out->normal=m->nrm;out->uv=m->uv;
    out->blend_index=m->bi;out->blend_weight=m->bw;out->index=m->idx;
    return 1;
}
int psx_openxr_hand_joints(PSXModOpenXRHandJoints *out) {
    if(!out || out->struct_size!=sizeof *out)return 0;
    *out=s_joints;out->struct_size=sizeof *out;
    return 1;
}
int psx_openxr_hands(PSXModOpenXRHands *out) {
    if(!out || out->struct_size!=sizeof *out)return 0;
    if(s_hands_override){*out=s_injected_hands;return 1;}
    *out=s_hands;out->struct_size=sizeof *out;out->age_ms=UINT32_MAX;
#if defined(PSX_OPENXR)
    if(s_hands_sample_ms) {
        uint64_t now=psx_host_mono_ms();
        uint64_t age=now>=s_hands_sample_ms?now-s_hands_sample_ms:UINT32_MAX;
        out->age_ms=age<UINT32_MAX?(uint32_t)age:UINT32_MAX;
    }
#endif
    return 1;
}
int psx_openxr_interp_gate(uint32_t compiled,uint32_t enabled,uint32_t initialized,uint32_t running) {
    return compiled && enabled && initialized && running;
}
int psx_openxr_session_active(void) {
    return psx_openxr_interp_gate(s_stats.compiled,s_stats.enabled,s_stats.initialized,s_stats.running);
}
void psx_openxr_stats(PSXOpenXRStats *out) {
    if(out){*out=s_stats;
#if defined(PSX_OPENXR)
        out->compiled=1;
#endif
    }
}
void psx_openxr_recenter(void) {
    hands_clear();
#if defined(PSX_OPENXR)
    s_origin_valid=0;s_origin_inversion_streak=0;
    s_stats.origin_inversion_streak=s_stats.origin_ambiguous=0;
#endif
}
#if defined(PSX_OPENXR)
/* Leave a running session the way the spec asks before destroying it:
 * request exit, keep an empty frame loop going so the runtime can move to
 * STOPPING (events() then calls xrEndSession), bounded by a deadline.
 * Destroying a running session outright left the process hanging on exit
 * under Virtual Desktop's runtime (VDXR, 2026-11-20). */
static void xr_graceful_exit(void) {
    if(!s_session || !s_stats.running)return;
    if(xrRequestExitSession(s_session)!=XR_SUCCESS)return;
    uint64_t deadline=psx_host_mono_ms()+1500u;
    while(s_stats.running && psx_host_mono_ms()<deadline) {
        if(!events() || !s_stats.running)break;
        XrFrameWaitInfo wi={XR_TYPE_FRAME_WAIT_INFO};XrFrameState fs={XR_TYPE_FRAME_STATE};
        if(xrWaitFrame(s_session,&wi,&fs)!=XR_SUCCESS)break;
        XrFrameBeginInfo bi={XR_TYPE_FRAME_BEGIN_INFO};
        if(XR_FAILED(xrBeginFrame(s_session,&bi)))break;
        XrFrameEndInfo ei={XR_TYPE_FRAME_END_INFO};ei.displayTime=fs.predictedDisplayTime;
        ei.environmentBlendMode=XR_ENVIRONMENT_BLEND_MODE_OPAQUE;ei.layerCount=0;ei.layers=NULL;
        if(xrEndFrame(s_session,&ei)!=XR_SUCCESS)break;
    }
    (void)events();
}
#endif
int psx_openxr_haptic(uint32_t hand,float amplitude,uint32_t duration_ms) {
#if defined(PSX_OPENXR)
    if(hand>1 || !s_session || !s_haptic || !s_stats.running || !isfinite(amplitude))return 0;
    if(amplitude<0)amplitude=0;
    if(amplitude>1)amplitude=1;
    XrHapticVibration v={XR_TYPE_HAPTIC_VIBRATION};
    v.amplitude=amplitude;v.duration=(XrDuration)duration_ms*1000000;v.frequency=XR_FREQUENCY_UNSPECIFIED;
    XrHapticActionInfo info={XR_TYPE_HAPTIC_ACTION_INFO};info.action=s_haptic;info.subactionPath=s_hand[hand];
    return xrApplyHapticFeedback(s_session,&info,(const XrHapticBaseHeader*)&v)==XR_SUCCESS;
#else
    (void)hand;(void)amplitude;(void)duration_ms;return 0;
#endif
}
void psx_openxr_shutdown(void) {
#if defined(PSX_OPENXR)
    if(s_stats.frame_open)psx_openxr_end(0,NULL);
    xr_graceful_exit();
    xr_destroy();
    s_reset_pending=s_exit_pending=s_recovering=0;s_reset_stage=NULL;s_retry_ms=0;
#endif
    for(int i=0;i<2;i++){free(s_surface_pixels[i]);s_surface_pixels[i]=NULL;s_surface_capacity[i]=0;}
    s_surface_mask=0;
    s_stats.enabled=0;
    s_quad_distance=s_quad_width=s_quad_height=0;
    memset(&s_input,0,sizeof s_input);
    hands_clear();
    s_input_override=0;
    s_hands_override=0;memset(&s_injected_hands,0,sizeof s_injected_hands);
}
int psx_openxr_enable(int enabled) {
    psx_openxr_shutdown();memset(&s_stats,0,sizeof s_stats);
#if defined(PSX_OPENXR)
    s_stats.compiled=1;s_stats.enabled=enabled!=0;s_stats.stage=enabled?"requested":"off";return 1;
#else
    s_stats.stage="not_compiled";return !enabled;
#endif
}
int psx_openxr_view(uint32_t eye,PSXModRenderView *out) {
#if defined(PSX_OPENXR)
    if(s_stats.frame_open && s_stats.tracking && out && eye<2){*out=s_render[eye];return 1;}
#endif
    return 0;
}
int psx_openxr_input(PSXModOpenXRInput *out) {
    if(!out || out->struct_size!=sizeof *out)return 0;
    uint64_t seq=s_input.sequence+1;
    memset(&s_input,0,sizeof s_input);s_input.struct_size=sizeof s_input;s_input.sequence=seq;
    if(s_input_override) {
        s_input=s_injected;s_input.sequence=seq;s_input.synthetic=1;
        for(int e=0;e<2;e++) if(!s_input.focused || !s_input.active[e])
            s_input.stick[e][0]=s_input.stick[e][1]=0;
        for(int e=0;e<2;e++) {
            if(!s_input.focused || !s_input.trigger_active[e])s_input.trigger[e]=0;
            if(!s_input.focused || !s_input.squeeze_active[e])s_input.squeeze[e]=0;
            s_input.buttons[e] &= s_input.focused ? s_input.buttons_active[e] : 0;
        }
        if(!s_input.focused)hands_clear();
        *out=s_input;return 1;
    }
#if defined(PSX_OPENXR)
    if(s_stats.enabled && s_stats.initialized && !s_stats.frame_open && events() && s_stats.running) {
        if(sync_actions() && s_stats.state==XR_SESSION_STATE_FOCUSED) {
            PSXModOpenXRInput current=s_input;current.focused=1;int valid=1;
            for(int e=0;e<2;e++) {
                XrActionStateGetInfo get={XR_TYPE_ACTION_STATE_GET_INFO};get.action=s_stick;get.subactionPath=s_hand[e];
                XrActionStateVector2f state={XR_TYPE_ACTION_STATE_VECTOR2F};
                if(!check(xrGetActionStateVector2f(s_session,&get,&state),"stick_state")) { valid=0; break; }
                current.active[e]=state.isActive;
                if(state.isActive) {
                    current.stick[e][0]=state.currentState.x;current.stick[e][1]=state.currentState.y;
                    for(int a=0;a<2;a++) if(!isfinite(current.stick[e][a]) || fabsf(current.stick[e][a])>1) valid=0;
                }
                XrAction analog[2]={s_trigger,s_squeeze};
                for(int a=0;a<2;a++) {
                    get.action=analog[a];
                    XrActionStateFloat value={XR_TYPE_ACTION_STATE_FLOAT};
                    if(!check(xrGetActionStateFloat(s_session,&get,&value),"analog_action_state")) { valid=0;break; }
                    float v=value.isActive?value.currentState:0;
                    if(!isfinite(v) || v<0 || v>1) { valid=0;break; }
                    if(a==0){current.trigger_active[e]=value.isActive;current.trigger[e]=v;}
                    else {current.squeeze_active[e]=value.isActive;current.squeeze[e]=v;}
                }
                for(int a=0;a<4;a++) {
                    get.action=s_click[a];
                    XrActionStateBoolean value={XR_TYPE_ACTION_STATE_BOOLEAN};
                    if(!check(xrGetActionStateBoolean(s_session,&get,&value),"click_action_state")) { valid=0;break; }
                    if(value.isActive) {
                        current.buttons_active[e] |= 1u<<a;
                        if(value.currentState)current.buttons[e] |= 1u<<a;
                    }
                }
                if(!valid)break;
            }
            if(valid)s_input=current;
        }
    }
    /* events()/sync may have flagged a loss: tear down, return neutral input. */
    (void)service_loss();
#endif
    if(!s_input.focused)hands_clear();
    *out=s_input;return 1; /* Always a fresh neutral sample when unavailable. */
}
int psx_openxr_begin(int width,int height,double units) {
#if defined(PSX_OPENXR)
    if(!s_stats.enabled || s_stats.frame_open)return 0;
    s_surface_mask=0;
    s_quad_distance=s_quad_width=s_quad_height=0;
    s_pair_id=s_pair_cycle=s_native_frame=0;s_source=0;
    hands_clear();
    s_stats.units_per_meter=units;
    if(!s_stats.initialized) {
        if(s_retry_ms) { /* backing off after a loss: flat until the retry time */
            if(psx_host_mono_ms()<s_retry_ms)return 0;
            s_retry_ms=0;
        }
        if(!initialize()) {
#if defined(__ANDROID__)
            fprintf(stdout,"psxrecomp: OpenXR start-up stopped at: %s\n",s_stats.stage?s_stats.stage:"?");
#endif
            if(s_recovering) {
                /* The runtime/headset is still gone after a loss. Stay enabled,
                 * release the partial stack and try again later; the failure
                 * stays inspectable through the stats. */
                xr_destroy();s_retry_ms=psx_host_mono_ms()+XR_RETRY_MS;return 0;
            }
            /* Retain the failed producer/result. Avoid repeated costly startup until
             * explicit re-enable; a disconnected headset is an inspectable state. */
            psx_openxr_shutdown();return 0;
        }
        s_recovering=0;
    }
    if(!events() || !s_stats.running){(void)service_loss();return 0;}
    /* Open this frame's trace record before the blocking waits. The record is
     * committed by psx_openxr_end(); the two failure returns below commit it
     * themselves so a lost session cannot leave a record open forever. */
    psx_frame_trace_open();
    /* Cleared so a frame that never reaches xrLocateViews (no render request,
     * lost tracking, bad view maths) cannot report a stale pose age. */
    s_pose_sample_ticks=0;
    uint64_t trace_t = gl_renderer_perf_ticks();
    XrFrameWaitInfo wait={XR_TYPE_FRAME_WAIT_INFO};XrFrameState fs={XR_TYPE_FRAME_STATE};
    if(!check(xrWaitFrame(s_session,&wait,&fs),"wait_frame")){
        psx_frame_trace_add(PSX_TRACE_XR_WAIT_FRAME, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
        /* Session-level submission failure: the stereo cadence context is gone,
         * so drop any stale pair-time estimate rather than carrying it into the
         * recovery attempt. */
        psx_mod_stereo_reset_budget();
        (void)service_loss();psx_frame_trace_commit();return 0;}
    psx_frame_trace_add(PSX_TRACE_XR_WAIT_FRAME, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
    s_stats.waits++;s_time=fs.predictedDisplayTime;s_stats.predicted_time=(uint64_t)s_time;
    /* The runtime's own clock between submissions. This is the cadence the
     * compositor saw, independent of our frame timing: delta / display period
     * is how many granted periods one presented frame consumed. */
    s_display_delta_us=0;
    if(s_prev_display_valid && s_time>s_prev_display_time) {
        uint64_t d=(uint64_t)(s_time-s_prev_display_time)/1000u;
        s_display_delta_us=d>0xFFFFFFFFull?0xFFFFFFFFu:(uint32_t)d;
    }
    s_prev_display_time=s_time;s_prev_display_valid=1;
    /* Record the runtime display period with its name for cadence analysis. */
    /* What the runtime wants us to hit is the only true deadline for the pair.
     * Hand it to stereo shedding (PSX VBlank is a guest redraw budget, not a
     * compositor deadline) and record it with the runtime name so a trace can be
     * read without guessing the headset's refresh. */
    psx_mod_stereo_set_budget_ms(fs.predictedDisplayPeriod>0?(double)fs.predictedDisplayPeriod/1e6:0.0);
    psx_frame_trace_session(s_stats.runtime,
                            fs.predictedDisplayPeriod>0?(uint32_t)(fs.predictedDisplayPeriod/1000):0u);
    trace_t = gl_renderer_perf_ticks();
    XrFrameBeginInfo begin={XR_TYPE_FRAME_BEGIN_INFO};
    if(!check(xrBeginFrame(s_session,&begin),"begin_frame")){
        psx_frame_trace_add(PSX_TRACE_XR_BEGIN_FRAME, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
        psx_mod_stereo_reset_budget();
        (void)service_loss();psx_frame_trace_commit();return 0;}
    psx_frame_trace_add(PSX_TRACE_XR_BEGIN_FRAME, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
    s_stats.frame_open=1;s_stats.tracking=0;
    if(!fs.shouldRender){psx_openxr_end(0,NULL);return 0;}
    XrViewLocateInfo loc={XR_TYPE_VIEW_LOCATE_INFO};loc.viewConfigurationType=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    loc.displayTime=s_time;loc.space=s_space;
    XrViewState vs={XR_TYPE_VIEW_STATE};uint32_t count=0;
    memset(s_views,0,sizeof s_views);s_views[0].type=s_views[1].type=XR_TYPE_VIEW;
    uint64_t locate_t=gl_renderer_perf_ticks();
    int located=check(xrLocateViews(s_session,&loc,&vs,2,&count,s_views),"locate_views");
    psx_frame_trace_add(PSX_TRACE_XR_LOCATE, xr_trace_us(gl_renderer_perf_ticks()-locate_t));
    s_pose_sample_ticks=gl_renderer_perf_ticks();
    uint64_t prep_t=s_pose_sample_ticks;
    s_stats.view_flags=(uint64_t)vs.viewStateFlags;
    if(!located || count!=2 ||
       (vs.viewStateFlags&(XR_VIEW_STATE_ORIENTATION_VALID_BIT|XR_VIEW_STATE_POSITION_VALID_BIT))!=
       (XR_VIEW_STATE_ORIENTATION_VALID_BIT|XR_VIEW_STATE_POSITION_VALID_BIT)) {
        s_stats.stage="tracking_invalid";psx_openxr_end(0,NULL);return 0;
    }
    double q[2][4],p[2][3];
    for(int i=0;i<2;i++) {
        XrPosef *v=&s_views[i].pose;
        q[i][0]=v->orientation.x;q[i][1]=v->orientation.y;q[i][2]=v->orientation.z;q[i][3]=v->orientation.w;
        p[i][0]=v->position.x;p[i][1]=v->position.y;p[i][2]=v->position.z;
    }
    if(xr_origin_reference_reset_due(s_origin_reset_time!=0,s_origin_reset_time,s_time)) {
        s_origin_valid=0;s_origin_reset_time=0;s_origin_inversion_streak=0;
    }
    if(!s_origin_valid) {
        memcpy(s_oq,q[0],sizeof s_oq);
        for(int j=0;j<3;j++)s_op[j]=(p[0][j]+p[1][j])*.5;
        s_origin_valid=1;
        s_origin_inversion_streak=0;
    }
    /* A sustained roll can be a legitimate headset pose. Report the ambiguous
     * case without changing the reference; explicit recenter or the OpenXR
     * reference-space change event owns origin recovery. */
    double origin_up_dot=1.0;
    s_origin_inversion_streak=xr_origin_observe_inversion(
        s_origin_inversion_streak,s_origin_valid,s_oq,q[0],&origin_up_dot);
    s_stats.origin_up_dot=(float)origin_up_dot;
    s_stats.origin_inversion_streak=s_origin_inversion_streak;
    s_stats.origin_ambiguous=s_origin_inversion_streak>=10u;
    double separation=0;
    for(int j=0;j<3;j++)separation+=(p[1][j]-p[0][j])*(p[1][j]-p[0][j]);
    s_stats.ipd_m=(float)sqrt(separation);
#ifndef PSX_NO_DEBUG_TOOLS
    psx_openxr_pose_bob(q, p);   /* test-only synthetic head motion; no-op unless armed */
#endif
    for(int i=0;i<2;i++) {
        XrFovf *v=&s_views[i].fov;
        double f[4]={v->angleLeft,v->angleRight,v->angleUp,v->angleDown};
        if(!vr_pose_to_view(q[i],p[i],s_oq,s_op,f,units,width,height,&s_render[i])) {
            s_stats.stage="view_math";psx_openxr_end(0,NULL);return 0;
        }
    }
    for(int i=0;i<2;i++) {
        for(int j=0;j<3;j++)s_stats.pose[i][j]=(float)p[i][j];
        for(int j=0;j<4;j++)s_stats.pose[i][j+3]=(float)q[i][j];
        s_stats.fov[i][0]=s_views[i].fov.angleLeft;s_stats.fov[i][1]=s_views[i].fov.angleRight;
        s_stats.fov[i][2]=s_views[i].fov.angleUp;s_stats.fov[i][3]=s_views[i].fov.angleDown;
        s_stats.view[i]=s_render[i];
    }
    hands_locate(s_stats.state==XR_SESSION_STATE_FOCUSED && sync_actions());
    psx_frame_trace_add(PSX_TRACE_HOST_PREP, xr_trace_us(gl_renderer_perf_ticks()-prep_t));
    s_stats.tracking=1;s_stats.stage="located";return 1;
#endif
    return 0;
}
int psx_openxr_quad(double distance,double width,double height) {
    if(!isfinite(distance) || !isfinite(width) || !isfinite(height))return 0;
    if(distance!=0 && (distance<.25 || distance>20 || width<=0 || width>20 || height<=0 || height>20))return 0;
#if defined(PSX_OPENXR)
    if(!s_stats.frame_open || !s_stats.tracking)return 0;
    if(distance && !s_view_space) {
        XrReferenceSpaceCreateInfo space={XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        space.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_VIEW;space.poseInReferenceSpace.orientation.w=1;
        if(!check(xrCreateReferenceSpace(s_session,&space,&s_view_space),"quad_view_space"))return 0;
    }
    s_quad_distance=distance;s_quad_width=distance?width:0;s_quad_height=distance?height:0;return 1;
#else
    return 0;
#endif
}
/* Dynamic resolution (psxrecomp573, 2026-11-24): the host scene may draw into
 * a smaller part of each eye image (psx_openxr_set_render_scale); the copy
 * reports the part it drew (psx_openxr_eye_extent) and the projection views
 * hand only that rectangle to the compositor. Full size unless a copy says. */
static float s_render_scale=1.0f;
static int s_eye_ext[2][2];
void psx_openxr_set_render_scale(float scale) {
    if(!(scale>=0.5f))scale=0.5f;if(scale>1.0f)scale=1.0f;
    s_render_scale=scale;
}
float psx_openxr_render_scale(void){return s_render_scale;}
void psx_openxr_eye_extent(uint32_t eye,int w,int h) {
    if(eye<2 && w>0 && h>0){s_eye_ext[eye][0]=w;s_eye_ext[eye][1]=h;}
}
int psx_openxr_end(int keep,PSXOpenXRCopy copy) {
#if defined(PSX_OPENXR)
    if(!s_stats.frame_open)return 0;
    for(int i=0;i<2;i++){s_eye_ext[i][0]=s_w[i];s_eye_ext[i][1]=s_h[i];}
    /* Defensive open: normally psx_openxr_begin() already opened the record. */
    psx_frame_trace_open();
    uint64_t trace_t;
    XrCompositionLayerProjectionView pv[2]={{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
    int complete=keep && copy && s_stats.tracking;
    int quad=s_quad_distance>0;
    for(int eye=0;eye<(quad?1:2) && complete;eye++) {
        uint32_t index=0;
        trace_t = gl_renderer_perf_ticks();
        XrSwapchainImageAcquireInfo ai={XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
        if(!check(xrAcquireSwapchainImage(s_chain[eye],&ai,&index),"acquire")){complete=0;break;}
        psx_frame_trace_add(PSX_TRACE_XR_ACQUIRE, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
        XrSwapchainImageWaitInfo wi={XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};wi.timeout=1000000000;
        trace_t = gl_renderer_perf_ticks();
        XrResult wr=xrWaitSwapchainImage(s_chain[eye],&wi);
        /* A timeout leaves ownership acquired: wait again (bounded) before
         * releasing. Only XR_SUCCESS means the image is writable; the timeout
         * code is a success code and must not be mistaken for it. */
        if(wr==XR_TIMEOUT_EXPIRED){wi.timeout=2000000000;wr=xrWaitSwapchainImage(s_chain[eye],&wi);}
        psx_frame_trace_add(PSX_TRACE_XR_WAIT_IMAGE, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
        int writable=check(wr,"wait_image") && wr==XR_SUCCESS;
        if(writable) {
            trace_t = gl_renderer_perf_ticks();
            if(index>=s_count[eye] || !copy(eye,s_images[eye][index].image,s_w[eye],s_h[eye])) {
                s_stats.stage="copy_image";s_stats.failures++;complete=0;
            }
            psx_frame_trace_add(PSX_TRACE_XR_COPY, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
            trace_t = gl_renderer_perf_ticks();
            XrSwapchainImageReleaseInfo ri={XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            if(!check(xrReleaseSwapchainImage(s_chain[eye],&ri),"release"))complete=0;
            psx_frame_trace_add(PSX_TRACE_XR_RELEASE, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
        } else {
            /* Acquired but never writable. Release it best-effort so the acquire
             * count cannot leak; a runtime that refuses (the wait never completed)
             * leaves this swapchain unusable for the next acquire, so rebuild the
             * whole stack rather than fail every later frame. */
            XrSwapchainImageReleaseInfo ri={XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            (void)xrReleaseSwapchainImage(s_chain[eye],&ri);
            if(wr==XR_TIMEOUT_EXPIRED){s_stats.stage=s_stats.last_failure="wait_image_timeout";s_stats.failures++;}
            if(!s_reset_pending){s_reset_pending=1;s_reset_stage="swapchain_wait_failed";}
            complete=0;
        }
        pv[eye].pose=s_views[eye].pose;pv[eye].fov=s_views[eye].fov;
        pv[eye].subImage.swapchain=s_chain[eye];
        pv[eye].subImage.imageRect.extent.width=s_eye_ext[eye][0]<=s_w[eye]?s_eye_ext[eye][0]:s_w[eye];
        pv[eye].subImage.imageRect.extent.height=s_eye_ext[eye][1]<=s_h[eye]?s_eye_ext[eye][1]:s_h[eye];
    }
    XrCompositionLayerProjection layer={XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    layer.space=s_space;layer.viewCount=2;layer.views=pv;
    XrCompositionLayerQuad surface={XR_TYPE_COMPOSITION_LAYER_QUAD};
    surface.space=s_view_space;surface.eyeVisibility=XR_EYE_VISIBILITY_BOTH;
    surface.subImage=pv[0].subImage;surface.pose.orientation.w=1;
    surface.pose.position.z=(float)-s_quad_distance;
    surface.size.width=(float)s_quad_width;surface.size.height=(float)s_quad_height;
    const XrCompositionLayerBaseHeader *layers[]={quad?(void*)&surface:(void*)&layer};
    XrFrameEndInfo end={XR_TYPE_FRAME_END_INFO};end.displayTime=s_time;
    end.environmentBlendMode=XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    end.layerCount=complete?1:0;end.layers=complete?layers:NULL;
    trace_t = gl_renderer_perf_ticks();
    int ok=check(xrEndFrame(s_session,&end),"end_frame");s_stats.frame_open=0;
    psx_frame_trace_add(PSX_TRACE_XR_END_FRAME, xr_trace_us(gl_renderer_perf_ticks()-trace_t));
    if(s_pose_sample_ticks)
        psx_frame_trace_add(PSX_TRACE_POSE_AGE,
                            xr_trace_us(gl_renderer_perf_ticks()-s_pose_sample_ticks));
    psx_frame_trace_xr_cadence(s_display_delta_us, complete);
    if(ok && complete) {
        s_stats.submitted_session_generation=s_stats.session_generation;
        s_stats.submitted_surfaces=quad?0:((s_surface_mask&1)!=0)+((s_surface_mask&2)!=0);
        s_stats.submitted++;s_stats.submitted_pair_id=s_pair_id;
        s_stats.submitted_guest_cycle=s_pair_cycle;s_stats.submitted_predicted_time=(uint64_t)s_time;
        s_stats.submitted_layer=quad?2:1;
        s_stats.submitted_source=s_source;s_stats.submitted_native_frame=s_native_frame;
        if(s_source==2)s_stats.native_submitted++;
        if(quad){s_stats.quad_submitted++;s_stats.quad_distance_m=s_quad_distance;
            s_stats.quad_width_m=s_quad_width;s_stats.quad_height_m=s_quad_height;}
        /* A fresh stereo pair already labelled the record source 1; only
         * annotate native presentation when nothing else claimed it. */
        psx_frame_trace_source(2);
    } else {
        s_stats.empty++;s_stats.submitted_surfaces=0;
        psx_frame_trace_flag(PSX_FRAME_TRACE_XR_EMPTY);
        psx_frame_trace_flag(PSX_FRAME_TRACE_STALE_PAIR);
        psx_frame_trace_source(2);
    }
    if(ok && complete)s_stats.stage="submitted";
    s_quad_distance=s_quad_width=s_quad_height=0;
    psx_frame_trace_commit();
    (void)service_loss(); /* after the frame is closed: lost session / bad swapchain */
    return ok && complete;
#else
    return 0;
#endif
}

int psx_openxr_eye_scene(uint32_t eye,double n,double f,float view[16],float proj[16],
                         double eye_p[3],double eye_q[4],uint32_t *origin_valid,
                         double origin_p[3],double origin_q[4],uint64_t *predicted_time) {
#if defined(PSX_OPENXR)
    if(!s_stats.frame_open || !s_stats.tracking || eye>1 || !view || !proj)return 0;
    if(!(n>0) || !(f>n) || !isfinite(f))return 0;
    const XrView *v=&s_views[eye];
    double x=v->pose.orientation.x,y=v->pose.orientation.y,z=v->pose.orientation.z,w=v->pose.orientation.w;
    double len=sqrt(x*x+y*y+z*z+w*w);
    if(!(len>.5) || !isfinite(len))return 0;
    x/=len;y/=len;z/=len;w/=len;
    /* Rotation of the eye in LOCAL space (row r, column c). */
    double r[3][3]={{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)},
                    {2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)},
                    {2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}};
    double t[3]={v->pose.position.x,v->pose.position.y,v->pose.position.z};
    /* view = inverse(T*R) = R^T * T^-1, column-major. */
    for(int c=0;c<3;c++)for(int rr=0;rr<3;rr++)view[c*4+rr]=(float)r[c][rr];
    view[3]=view[7]=view[11]=0;
    for(int rr=0;rr<3;rr++)view[12+rr]=(float)-(r[0][rr]*t[0]+r[1][rr]*t[1]+r[2][rr]*t[2]);
    view[15]=1;
    double tl=tan(v->fov.angleLeft),tr=tan(v->fov.angleRight);
    double tu=tan(v->fov.angleUp),td=tan(v->fov.angleDown);
    double wd=tr-tl,ht=tu-td;
    if(!(wd>0) || !(ht>0))return 0;
    memset(proj,0,16*sizeof *proj);
    proj[0]=(float)(2/wd);proj[8]=(float)((tr+tl)/wd);
    proj[5]=(float)(2/ht);proj[9]=(float)((tu+td)/ht);
    proj[10]=(float)(-(f+n)/(f-n));proj[11]=-1;proj[14]=(float)(-2*f*n/(f-n));
    if(eye_p){eye_p[0]=t[0];eye_p[1]=t[1];eye_p[2]=t[2];}
    if(eye_q){eye_q[0]=x;eye_q[1]=y;eye_q[2]=z;eye_q[3]=w;}
    if(origin_valid)*origin_valid=(uint32_t)s_origin_valid;
    if(origin_p)memcpy(origin_p,s_op,sizeof s_op);
    if(origin_q)memcpy(origin_q,s_oq,sizeof s_oq);
    if(predicted_time)*predicted_time=(uint64_t)s_time;
    return 1;
#else
    (void)eye;(void)n;(void)f;(void)view;(void)proj;(void)eye_p;(void)eye_q;
    (void)origin_valid;(void)origin_p;(void)origin_q;(void)predicted_time;
    return 0;
#endif
}

int psx_openxr_floor(float *y_m) {
#if defined(PSX_OPENXR)
    if(!s_stats.frame_open || !s_stage_space || !y_m)return 0;
    XrSpaceLocation loc={XR_TYPE_SPACE_LOCATION};
    if(xrLocateSpace(s_stage_space,s_space,s_time,&loc)!=XR_SUCCESS)return 0;
    if(!(loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT))return 0;
    float y=loc.pose.position.y;
    if(!isfinite(y) || fabsf(y)>10)return 0;
    *y_m=y;
    return 1;
#else
    (void)y_m;
    return 0;
#endif
}
int psx_openxr_surface(const PSXModOpenXRSurface *v) {
    if(!psx_openxr_surface_valid(v))return 0;
    if(!s_stats.frame_open || !s_stats.tracking || s_quad_distance>0)return 0;
    size_t bytes=(size_t)v->width_px*v->height_px*4;
    if(s_surface_capacity[v->id]<bytes) {
        uint8_t *buffer=realloc(s_surface_pixels[v->id],bytes);
        if(!buffer){s_stats.surface_failures++;return 0;}
        s_surface_pixels[v->id]=buffer;s_surface_capacity[v->id]=bytes;
    }
    memcpy(s_surface_pixels[v->id],v->rgba,bytes);
    s_surfaces[v->id]=*v;s_surfaces[v->id].rgba=s_surface_pixels[v->id];
    s_surface_mask|=1u<<v->id;s_stats.surface_uploads++;return 1;
}
int psx_openxr_surface_get(uint32_t id,PSXModOpenXRSurface *out) {
    if(!out || id>=2 || !(s_surface_mask&(1u<<id)) || !s_stats.frame_open || s_quad_distance>0)return 0;
    *out=s_surfaces[id];return 1;
}
