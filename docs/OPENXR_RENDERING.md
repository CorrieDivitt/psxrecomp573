# Experimental Win32/OpenGL OpenXR backend

Build with `-DPSX_OPENXR=ON` to opt in. The default is OFF: ordinary builds
do not fetch the SDK or require an active headset runtime. The enabled build
uses the official OpenXR-SDK source archive for annotated tag object
`b76b80adaf65ac3ad6cc1ce61974fb29a5d02352` (SHA256-pinned in `third_party/deps.manifest`;
same vendored/offline resolution as libchdr) and currently supports Win32/OpenGL. This is a PC headset backend; it does not
provide an Android/standalone Quest build.

## Opting in at run time: `PSX_OPENXR_ENABLE`

The CMake option `PSX_OPENXR` only compiles the backend. A compiled-in build
still creates the usual OpenGL 3.3 core context. Setting the environment variable
`PSX_OPENXR_ENABLE=1` makes an XR build request an OpenGL **4.6** core context
instead (the OpenXR OpenGL runtime requirements need it); the runtime logs one
line to stderr when this changes the context version. It has no effect in a build
without `-DPSX_OPENXR=ON`. The older spelling `PSX_OPENXR=1` (identical to
the CMake option name) still works but is deprecated, logs a note, and will be
removed. The variable only selects the context; the game plugin still enables
XR with `psx_mod_openxr_enable`.

## Threading

Every OpenXR entry point runs on the emulation/main thread that owns the GL
context: plugin hooks (`psx_mod_openxr_*`), the GL present path
(`gl_swap_with_osd` and the interpolation/hold presents it serves) and TCP
command handlers, which the debug server executes at its `debug_server_poll`
safe point on that same thread (its I/O thread only queues requests). There is
no second GL or XR thread, so the module has no locks. New code must keep it
that way or add marshalling.

## Session loss and recovery

`XR_ERROR_SESSION_LOST`/`XR_ERROR_INSTANCE_LOST` (from any call), session
`LOSS_PENDING` and instance loss tear the whole XR stack down at the end of
the failing call and fall back to the flat present. The backend stays enabled,
retries initialisation every few seconds, and counts `losses` in
`openxr_stats`. A session that reaches `EXITING` (the user closed the headset
app) is torn down and left disabled until `openxr_control enable=1` /
`psx_mod_openxr_enable(1)`. A swapchain image whose wait fails is treated as an
unrecoverable swapchain and takes the same path.

## Frame ownership and stereo submission

The game plugin enables the backend with `psx_mod_openxr_enable`, then calls
`psx_mod_openxr_begin(width,height,units_per_meter)` at a host frame boundary.
Begin waits/begins the XR frame and locates both eyes at its predicted time.
`psx_mod_openxr_view` returns the scoped rigid transform and asymmetric
projection for each eye. The plugin supplies its own draw-only callbacks to
the [atomic stereo renderer](STEREO_RENDERING.md).

`psx_mod_openxr_end(pair_rendered)` submits only the fresh complete pair from
that frame. A failed, refused or shed redraw submits zero layers, closing the
frame without recycling an old pair. Private swapchains receive eye textures;
guest execution, simulation time and guest TR registers are not owned by XR.
`psx_mod_openxr_recenter` establishes the shared pose origin. World scale,
native camera/body mapping, aiming and gameplay policy belong to the plugin.
An inverted relative head pose can be a deliberate sustained roll, so the
backend reports its up-dot and inversion streak through `openxr_stats` without
changing the origin. Explicit recenter and OpenXR reference-space change events
own origin changes.

## Offline actions and controller snapshots

Touch stick, trigger, squeeze and click actions are sampled at the normal
offline input boundary, never during eye replay. `psx_mod_openxr_input` exposes
activity/focus, sequence and synthetic metadata. Unfocused/inactive actions
are neutral; the game chooses how to map them through the separate trusted
controller-source API. Default flat input registers no XR source.

`psx_mod_openxr_hands` is read-only and safe during replay. It reports grip/aim
poses for both hands, activity, OpenXR validity/tracking flags, common origin,
predicted time, sequence and age. A cached pose is not a promise that tracking
is currently valid. Focus loss, disable and shutdown clear samples. Relative
controller transforms use the same metric pose math and origin as the eyes.

TCP `openxr_stats`, `openxr_views`, `openxr_input` and `openxr_hands` expose the
backend. `openxr_control` enables/recenters it. Synthetic `openxr_input_override`
and `openxr_hands_override` exist only with debug tools enabled and label their
samples. Their handlers and registrations are excluded by PSX_NO_DEBUG_TOOLS.

## Menus and native startup surfaces

`psx_mod_openxr_quad(distance_m,width_m,height_m)` chooses a frame-local menu
quad after pair begin. The request resets each frame; normal gameplay resumes
projection submission. Applications choose when their identical-eye menus
should become a comfortable quad. Distance must be 0.25..20m and dimensions
positive/bounded; values must be finite.

`psx_mod_openxr_native_surface(distance_m,width_m,units_per_meter)` permits
boot/menu/video presentation before any gameplay stereo hook. It persists
until disabled with distance zero. Width is positive and at most 10m, scale
1..65536, distance 0.25..20m; values must be finite. It refuses during replay,
an open XR frame, or compiled-out XR. Disable it before application pair begin.

Only a fresh native present arms submission. The letterboxed GL_BACK rectangle
is copied before host OSD, upright, to a VIEW-space quad preserving its aspect.
This path does not replay guest draws. Unchanged native frames still pump XR.
`openxr_stats.submitted_source` distinguishes stereo (1) and native (2).
Successful-submit counters and latched frame/cycle metadata describe the last
submission; they do not prove the layer currently visible. `video_info` exposes
the actual GL swap interval. The plugin can avoid a second desktop wait while
keeping the runtime's guest-speed deadline.

## Host-drawn scene: `psx_mod_openxr_scene`

**Thread and rate.** The scene runs on its own XR thread with a GL context
shared with the composing context, paced by `xrWaitFrame` at the headset's
refresh rate rather than the game's presents. The composing thread copies each
fresh native frame into one of three shared textures, fences it and publishes
it; the XR thread waits on that fence and draws with the newest frame. Drawing
at the game's 60 Hz instead made the runtime reproject every frame, which
smeared the screen badly on horizontal head turns (Virtual Desktop, Quest 3,
2026-11-20). The XR thread owns the OpenXR session: it samples input, locates
the controllers (`PSXModOpenXRSceneEye.hands`), and ends and destroys the
session when the scene is removed or the renderer shuts down
(`gl_renderer_xr_scene_stop`). Scene callbacks must not call
`psx_mod_openxr_hands` or `psx_mod_openxr_input` (they synchronise with the
render thread); `psx_mod_openxr_haptic` is safe there.

For titles whose VR presentation is a world the application builds itself (a
virtual arcade cabinet with the game on its screen, a room around a flat game),
`psx_mod_openxr_scene(draw,user,near_m,far_m)` registers a callback that draws
that world into the stereo projection layer, once per eye, at every fresh native
present (the same source and point as the native surface). There is no guest
replay and no change to guest timing.

Before the XR frame begins, the composed native viewport is copied into a
texture: display-encoded RGBA8, bottom row first, with the desktop
post-processing already applied. Inside the frame, for each eye the callback
receives a `PSXModOpenXRSceneEye`: an FBO bound for drawing whose colour is the
eye's swapchain image and whose depth is a private DEPTH24_STENCIL8
renderbuffer, the full-image viewport, column-major `view` (raw XR LOCAL metres
to eye) and `projection` (asymmetric GL clip from the eye's FOV and the given
near/far), the eye pose, the recentre origin (the same one
`PSXModOpenXRHands` reports), and the game texture with its size.
`GL_FRAMEBUFFER_SRGB` is disabled: write display-encoded values, or decode to
linear when `linear_target` is 1 (a runtime without an sRGB swapchain format).
The runtime restores the GL state its native renderer depends on afterwards.
Returning 0 from the callback submits no layer for that frame.

Controller actions for the scene's frame are sampled on the presenting
thread just before the XR frame opens and passed in `input` (the same sample
for both eyes). Use it rather than calling `psx_mod_openxr_input` from the
emulation thread: with the render thread on, such a call can arrive while a
frame is open, where it samples nothing and clears the cached controller
poses, so the scene sees no hands for that frame.

The scene excludes the native surface and the guest stereo path
(`psx_mod_openxr_begin` refuses while a scene is registered). Controllers come
from `psx_mod_openxr_hands`, sampled when the frame's views are located. The
layer is submitted at the native present rate, so a headset faster than the
game reprojects between game frames. Not yet validated on a headset (first use:
Fighting Mania VR, 2026-11-20).

## Haptics and session exit

A vibration output action is bound to `/user/hand/left/output/haptic` and
`/user/hand/right/output/haptic` for the Touch and KHR simple profiles.
`psx_mod_openxr_haptic(hand,amplitude,duration_ms)` applies it; call it on the
presenting thread (for example from the scene callback).

`psx_openxr_shutdown` ends a running session before destroying it: it requests
exit, runs empty frames until the runtime reaches STOPPING (where `xrEndSession`
is called), at most 1.5 s, and then destroys the objects. Destroying a running
session directly hung the process on exit under Virtual Desktop's runtime.

## Validation and limits

The original MoH alpha was accepted on Quest 3 via Virtual Desktop/VDXR for
startup/menu visibility, head tracking and mapped combat input. Menu quads and
native startup surfaces were exercised. This is historical acceptance of the
alpha checkpoint, not a new headset run of this rebased branch.

Fresh extraction checks cover action lifecycle/neutralization, pose math,
paired watchdog rollback, GTE identity/parallax/rigid/asymmetric projection,
and XR/debug-tool compile combinations. Controlled focus/reconnection tests,
other runtimes/headsets and fresh rebased headset acceptance remain pending.
Guest-frame replay cost does not guarantee headset-rate rendering. No claim
of independent headset-rate simulation, controller/barrel alignment or Android
support is made. The projection-scale prerequisite is runtime-only; no retail BIOS
regeneration is required. Bundled OpenBIOS is suitable for enabled builds.

The pin resolves to SDK commit `c15d38cb4bb10a5b7e075f74493ff13896e2597a`.
See [Third-Party Attribution](../THIRD_PARTY_ATTRIBUTION.md#openxr-sdk-loader---optional-pc-headset-backend)
for the Apache-2.0 SDK, MIT JsonCpp notices, cold-cache requirements and package
distribution. Consuming this feature requires a framework pin update and runtime
rebuild; it does not change guest code generation.

## Headset color candidate (2026-10-05)

`fix/vr-headset-color` corrects submission encoding. Native PSX/stereo textures
contain display-encoded RGB; the previous unconverted GL_RGBA8 swapchain made
the compositor interpret those values as linear. Prefer GL_SRGB8_ALPHA8, with
GL_FRAMEBUFFER_SRGB disabled during the copy and restored afterward. If only
GL_RGBA8 is available, the copy shader explicitly decodes sRGB instead.

Gameplay applies desktop post-gamma once before encoding/decoding. Native
GL_BACK already contains that adjustment and is copied without applying it
again. Neither path changes guest GPU lighting, textures or simulation.
`openxr_stats.swapchain_format` reports the selected GL enum (35907 sRGB,
32856 linear). The linear-only native fallback uses an intermediate copy.

`runtime/tests/run_openxr_color_gl.py` runs the actual private copy functions
and shader against source-owned RGB/grayscale ramps in hidden real GL. Eight
format/gamma/incoming-sRGB-state combinations pass for gameplay and native
copies, including vertical orientation, alpha and GL state restoration.

## Frame interpolation is off while a session runs

With frame interpolation on, each interpolated sub-present reaches gl_swap_with_osd and, with a native surface enabled, openxr_present_native -> psx_openxr_begin -> xrWaitFrame. XR would then run several times per guest frame (stacked waits, per-eye ghosting). So presentation-side interpolation is suspended whenever psx_openxr_session_active() is true (compiled, enabled, initialized and running; pure decision: psx_openxr_interp_gate). The compositor reprojects instead. History is reset on every transition and interpolation resumes when the session stops. In a default build, or when XR is not enabled or no session is running, the gate is constant false and interpolation is unchanged.
