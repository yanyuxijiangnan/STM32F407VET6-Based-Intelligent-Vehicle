#include "line_follow.h"
#include "ccd.h"
#include "motor.h"

/* ======================== line-follow params (road-tune macros) ======================== */
#define LINE_CENTER_PIXEL        64.0f
#define LINE_BASE_SPEED          38.0f
#define LINE_WZ_LIMIT            50.0f

/* |error|>= : one-side-zero pivot while line still visible */
#define LINE_PIVOT_ERR           25.0f
#define LINE_PIVOT_SPEED         40.0f
/* |error|>= but below PIVOT: soft base + PID diff */
#define LINE_SOFT_ERR            14.0f
#define LINE_SOFT_BASE_SCALE     0.6f

/* On-line: update s_turn_sign only after |e|>=thr for N consecutive frames */
#define LINE_PREDICT_ERR         10.0f
#define LINE_PREDICT_HOLD_FRAMES 3U

/* ---- lost-line recover ---- */
/* main loop ~20ms/frame (CCD + HAL_Delay(20)) */
#define LINE_SEARCH_SPEED        30.0f
#define LINE_SEARCH_WZ           50.0f
#define LINE_SEARCH_MAX_FRAMES   70U   /* ~1.4s locked-dir sweep then stop */

#define LINE_BACKUP_SPEED        28U
#define LINE_BACKUP_FRAMES       12U   /* short; only when no direction at all */
#define LINE_SWEEP_FRAMES        25U   /* ~0.50s per side */
#define LINE_COMMIT_FRAMES       4U    /* optional very short; prefer instant FOLLOW */
#define LINE_RECOVER_MAX_FRAMES  150U  /* ~3.0s total recover timeout */
#define LINE_FOLLOW_STABLE_FRAMES 8U   /* clear s_backup_used after stable follow */

PID line_steer_pid;

volatile float line_error = 0.0f;
volatile float line_wz = 0.0f;
volatile uint16_t line_vL = 0;
volatile uint16_t line_vR = 0;
volatile uint8_t line_enabled = 1;
volatile uint8_t line_state = LF_FOLLOW;

static float s_last_error = 0.0f;
static int8_t s_turn_sign = 0;       /* error>0 => +1: left slow / right drive pivot */
static int8_t s_hist_sign = 0;       /* last valid turn direction */
static int8_t s_search_sign = 0;     /* locked for SEARCH_LOCK / sweeps */
static int8_t s_predict_sign = 0;    /* COMMIT pivot direction */
static uint16_t s_predict_hold = 0U; /* consecutive |e|>=PREDICT frames */

static uint16_t s_phase_frames = 0U;
static uint16_t s_recover_frames = 0U;
static uint16_t s_follow_stable = 0U;
static uint8_t  s_sweep_round = 0U;
static uint8_t  s_pref_right = 0U;
static uint8_t  s_backup_used = 0U;  /* at most one BACKUP per recover episode */

static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float absf(float v)
{
  return (v < 0.0f) ? -v : v;
}

static int8_t signf_nonzero(float v)
{
  if (v > 0.0f) return 1;
  if (v < 0.0f) return -1;
  return 0;
}

static void remember_sign(int8_t s)
{
  if (s != 0)
  {
    s_hist_sign = s;
  }
}

static void apply_pivot(int8_t sign, float *vL, float *vR)
{
  if (sign > 0)
  {
    *vL = 0.0f;
    *vR = LINE_PIVOT_SPEED;
    line_wz = LINE_PIVOT_SPEED;
  }
  else if (sign < 0)
  {
    *vL = LINE_PIVOT_SPEED;
    *vR = 0.0f;
    line_wz = -LINE_PIVOT_SPEED;
  }
  else
  {
    *vL = LINE_BASE_SPEED;
    *vR = LINE_BASE_SPEED;
    line_wz = 0.0f;
  }
}

/* Locked-dir search: SEARCH_SPEED +/- SEARCH_WZ (clamps to one-side ~0 = pivot-like) */
static void apply_search_lock(int8_t sign, float *vL, float *vR)
{
  float wz;

  if (sign == 0)
  {
    *vL = LINE_SEARCH_SPEED;
    *vR = LINE_SEARCH_SPEED;
    line_wz = 0.0f;
    return;
  }

  wz = (float)sign * LINE_SEARCH_WZ;
  line_wz = wz;
  *vL = LINE_SEARCH_SPEED - wz;
  *vR = LINE_SEARCH_SPEED + wz;
}

static void enter_state(uint8_t st)
{
  line_state = st;
  s_phase_frames = 0U;
}

static int8_t resolve_lock_sign(void)
{
  int8_t lock;

  lock = s_turn_sign;
  if (lock == 0)
  {
    lock = s_hist_sign;
  }
  if (lock == 0)
  {
    lock = signf_nonzero(s_last_error);
  }
  return lock;
}

static void plan_sweep_order(void)
{
  int8_t lock;

  lock = resolve_lock_sign();
  if (lock == 0)
  {
    lock = s_search_sign;
  }

  if (lock > 0)
  {
    s_pref_right = 1U;
    s_search_sign = lock;
    enter_state(LF_SWEEP_R);
  }
  else
  {
    s_pref_right = 0U;
    if (lock < 0)
    {
      s_search_sign = lock;
    }
    enter_state(LF_SWEEP_L);
  }
}

static void enter_search_lock(int8_t sign)
{
  line_steer_pid.integral = 0.0f;
  s_recover_frames = 0U;
  s_sweep_round = 0U;
  s_predict_sign = 0;
  s_search_sign = sign;
  remember_sign(sign);
  enter_state(LF_SEARCH_LOCK);
}

static void enter_backup_once(void)
{
  line_steer_pid.integral = 0.0f;
  s_recover_frames = 0U;
  s_sweep_round = 0U;
  s_predict_sign = 0;
  s_pref_right = 0U;
  s_backup_used = 1U;
  enter_state(LF_BACKUP);
}

/* Lost line from FOLLOW: prefer locked SEARCH; backup only if no direction and unused */
static void on_line_lost_from_follow(void)
{
  int8_t lock;

  lock = resolve_lock_sign();
  if (lock != 0)
  {
    enter_search_lock(lock);
    return;
  }

  if (s_backup_used == 0U)
  {
    enter_backup_once();
    return;
  }

  /* already backed up once this episode: blind L/R sweep */
  line_steer_pid.integral = 0.0f;
  s_recover_frames = 0U;
  s_sweep_round = 0U;
  plan_sweep_order();
}

static void output_lr(float vL, float vR)
{
  vL = clampf(vL, 0.0f, (float)MAX_SPEED);
  vR = clampf(vR, 0.0f, (float)MAX_SPEED);
  line_vL = (uint16_t)vL;
  line_vR = (uint16_t)vR;
  Motor_SetLR(line_vL, line_vR);
}

static void update_turn_sign_stable(float abs_err, int8_t err_sign)
{
  if ((abs_err >= LINE_PREDICT_ERR) && (err_sign != 0))
  {
    if (s_predict_hold < 0xFFFFU)
    {
      s_predict_hold++;
    }
    if (s_predict_hold >= LINE_PREDICT_HOLD_FRAMES)
    {
      s_turn_sign = err_sign;
      remember_sign(err_sign);
    }
  }
  else
  {
    s_predict_hold = 0U;
  }
}

static void follow_with_line(float *vL, float *vR)
{
  float wz;
  float abs_err;
  float base;
  int8_t err_sign;

  wz = PID_location(&line_steer_pid, (float)CCD_Center);
  wz = clampf(wz, -LINE_WZ_LIMIT, LINE_WZ_LIMIT);
  line_error = line_steer_pid.error;
  abs_err = absf(line_error);
  err_sign = signf_nonzero(line_error);

  s_last_error = line_error;

  /* stable 3-frame predict (small bends still pivot via PIVOT_ERR path) */
  update_turn_sign_stable(abs_err, err_sign);

  if (abs_err >= LINE_PIVOT_ERR)
  {
    /* line still visible, large error: direct pivot, never backup */
    apply_pivot(err_sign, vL, vR);
    if (err_sign != 0)
    {
      /* hard commit direction on big visible bend */
      s_turn_sign = err_sign;
      remember_sign(err_sign);
      s_predict_hold = LINE_PREDICT_HOLD_FRAMES;
    }
  }
  else
  {
    base = LINE_BASE_SPEED;
    if (abs_err >= LINE_SOFT_ERR)
    {
      base = LINE_BASE_SPEED * LINE_SOFT_BASE_SCALE;
    }
    line_wz = wz;
    *vL = base - wz;
    *vR = base + wz;
  }
}

static int8_t make_predict_sign(int8_t sweep_side_sign)
{
  int8_t s = signf_nonzero(LINE_CENTER_PIXEL - (float)CCD_Center);
  if (s == 0)
  {
    s = sweep_side_sign;
  }
  if (s == 0)
  {
    s = s_search_sign;
  }
  if (s == 0)
  {
    s = s_turn_sign;
  }
  if (s == 0)
  {
    s = signf_nonzero(s_last_error);
  }
  if (s == 0)
  {
    s = s_hist_sign;
  }
  if (s == 0)
  {
    s = 1;
  }
  remember_sign(s);
  return s;
}

static void on_sweep_side_done(uint8_t just_finished_left)
{
  if (just_finished_left != 0U)
  {
    if (s_pref_right == 0U)
    {
      enter_state(LF_SWEEP_R);
    }
    else
    {
      s_sweep_round++;
      if (s_sweep_round >= 1U)
      {
        enter_state(LF_STOP);
      }
      else
      {
        enter_state(LF_SWEEP_R);
      }
    }
  }
  else
  {
    if (s_pref_right != 0U)
    {
      enter_state(LF_SWEEP_L);
    }
    else
    {
      s_sweep_round++;
      if (s_sweep_round >= 1U)
      {
        enter_state(LF_STOP);
      }
      else
      {
        enter_state(LF_SWEEP_L);
      }
    }
  }
}

/* Found line during recover: go FOLLOW immediately (skip long COMMIT tug) */
static void resume_follow_from_search(float *vL, float *vR)
{
  s_recover_frames = 0U;
  s_follow_stable = 0U;
  enter_state(LF_FOLLOW);
  follow_with_line(vL, vR);
  output_lr(*vL, *vR);
}

void LineFollow_Init(void)
{
  PID_init(&line_steer_pid, 0.9f, 0.0f, 0.04f, 0.05f, 200.0f);
  line_steer_pid.target = LINE_CENTER_PIXEL;
  line_enabled = 1;
  line_error = 0.0f;
  line_wz = 0.0f;
  line_vL = 0;
  line_vR = 0;
  line_state = LF_FOLLOW;
  s_last_error = 0.0f;
  s_turn_sign = 0;
  s_hist_sign = 0;
  s_search_sign = 0;
  s_predict_sign = 0;
  s_predict_hold = 0U;
  s_phase_frames = 0U;
  s_recover_frames = 0U;
  s_follow_stable = 0U;
  s_sweep_round = 0U;
  s_pref_right = 0U;
  s_backup_used = 0U;
  Motor_SetLR(0, 0);
}

void LineFollow_Update(void)
{
  float vL = 0.0f;
  float vR = 0.0f;

  if (line_enabled == 0U)
  {
    return;
  }

  /* ---------- LF_FOLLOW: on-line PID / soft / pivot ---------- */
  if (line_state == LF_FOLLOW)
  {
    if (CCD_LineValid != 0U)
    {
      s_recover_frames = 0U;
      if (s_follow_stable < 0xFFFFU)
      {
        s_follow_stable++;
      }
      /* after stable follow, allow one backup again next episode */
      if (s_follow_stable >= LINE_FOLLOW_STABLE_FRAMES)
      {
        s_backup_used = 0U;
      }
      follow_with_line(&vL, &vR);
      output_lr(vL, vR);
      return;
    }

    s_follow_stable = 0U;
    on_line_lost_from_follow();
  }

  /* recover timeout */
  if ((line_state != LF_FOLLOW) && (line_state != LF_STOP))
  {
    if (s_recover_frames < 0xFFFFU)
    {
      s_recover_frames++;
    }
    if (s_recover_frames > LINE_RECOVER_MAX_FRAMES)
    {
      enter_state(LF_STOP);
    }
  }

  if (s_phase_frames < 0xFFFFU)
  {
    s_phase_frames++;
  }

  switch (line_state)
  {
    case LF_BACKUP:
      /*
       * Ignore LineValid flicker while backing: do not bounce BACKUP<->FOLLOW.
       * Finish short backup, then sweep; s_backup_used blocks another BACKUP.
       */
      Motor_backward(LINE_BACKUP_SPEED);
      line_error = s_last_error;
      line_wz = 0.0f;
      line_vL = 0;
      line_vR = 0;

      if (s_phase_frames >= LINE_BACKUP_FRAMES)
      {
        /* after backup: if direction appeared, lock-search; else L/R sweep */
        {
          int8_t lock = resolve_lock_sign();
          if (lock != 0)
          {
            enter_search_lock(lock);
          }
          else
          {
            plan_sweep_order();
          }
        }
      }
      break;

    case LF_SEARCH_LOCK:
      /* locked direction, fixed SEARCH_WZ, no backup, ~70 frames then stop */
      if (CCD_LineValid != 0U)
      {
        resume_follow_from_search(&vL, &vR);
        break;
      }

      apply_search_lock(s_search_sign, &vL, &vR);
      line_error = s_last_error;
      output_lr(vL, vR);

      if (s_phase_frames >= LINE_SEARCH_MAX_FRAMES)
      {
        enter_state(LF_STOP);
      }
      break;

    case LF_SWEEP_L:
      if (CCD_LineValid != 0U)
      {
        s_predict_sign = make_predict_sign(-1);
        resume_follow_from_search(&vL, &vR);
        break;
      }

      apply_pivot(-1, &vL, &vR);
      line_error = s_last_error;
      output_lr(vL, vR);

      if (s_phase_frames >= LINE_SWEEP_FRAMES)
      {
        on_sweep_side_done(1U);
      }
      break;

    case LF_SWEEP_R:
      if (CCD_LineValid != 0U)
      {
        s_predict_sign = make_predict_sign(+1);
        resume_follow_from_search(&vL, &vR);
        break;
      }

      apply_pivot(+1, &vL, &vR);
      line_error = s_last_error;
      output_lr(vL, vR);

      if (s_phase_frames >= LINE_SWEEP_FRAMES)
      {
        on_sweep_side_done(0U);
      }
      break;

    case LF_COMMIT:
      /* kept for VOFA compatibility; normally unused (instant FOLLOW) */
      apply_pivot(s_predict_sign, &vL, &vR);
      if (CCD_LineValid != 0U)
      {
        line_error = LINE_CENTER_PIXEL - (float)CCD_Center;
      }
      else
      {
        line_error = s_last_error;
      }
      output_lr(vL, vR);

      if (s_phase_frames >= LINE_COMMIT_FRAMES)
      {
        enter_state(LF_FOLLOW);
      }
      break;

    case LF_STOP:
      line_error = 0.0f;
      line_wz = 0.0f;
      line_vL = 0;
      line_vR = 0;
      Motor_Coast();
      if (CCD_LineValid != 0U)
      {
        s_backup_used = 0U;
        s_follow_stable = 0U;
        enter_state(LF_FOLLOW);
        follow_with_line(&vL, &vR);
        output_lr(vL, vR);
      }
      break;

    default:
      enter_state(LF_FOLLOW);
      break;
  }
}