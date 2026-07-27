/*
 * tomcat_can_schema.h — TomCat control-interface schema (M1 task F1)
 *
 * SCOPE: DECLARATIONS ONLY — enums, structs, message IDs, bit masks, and the
 * fixed-point scaling constants that pin the wire units. NO control logic, NO
 * runtime functions. This header is the single source of truth for both
 * interface layers so host tools, the RT controller, and the smart drivers all
 * agree byte-for-byte.
 *
 *   Layer 1  SBC  <-> RT controller   (setpoints down / state up)  — logical structs
 *   Layer 2  RT   <-> smart drivers   (CAN-FD, ~5 Mbit/s, ~6-8 segments)
 *
 * References: docs/DESIGN_DECISIONS.md ADR-0002 (antagonistic + T_bias/AIC),
 * ADR-0004 (rotor sensor + OPEN tension front-end), ADR-0005 (distributed
 * CAN-FD drivers + RT/SBC split + 3 safety tiers). Field names line up with
 * kinematics TendonSolution (tension_*, motor_tension_*, motor_torque, t_bias).
 *
 * IMPLEMENTATION-NEUTRAL: no MCU, RTOS, endianness policy, or CRC polynomial is
 * fixed here (all -> ADR-0003/0005 follow-ups). Wire byte order is defined as
 * little-endian by convention below; revisit if a big-endian MCU is chosen.
 *
 * ⚠ GAPS (see INTERFACE.md "Open scaling / units"):
 *   - Tension LSB -> Newton transfer depends on the OPEN ADR-0004 method
 *     (load cell cal vs. current-estimate needing motor Kt + friction model).
 *   - current -> torque needs motor Kt (not yet chosen).
 *   - stiffness units assume cable-length reference (N/m); RT owns rad<->m.
 */

#ifndef TOMCAT_CAN_SCHEMA_H
#define TOMCAT_CAN_SCHEMA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Schema version — bump on any wire-layout change; echoed at handshake. */
#define TOMCAT_SCHEMA_VERSION_MAJOR 0u
#define TOMCAT_SCHEMA_VERSION_MINOR 1u

/* ------------------------------------------------------------------ *
 *  Fixed-point wire scaling (LSB definitions)                         *
 *  value_SI = wire_LSB * SCALE.  Chosen to fit compact CAN-FD frames. *
 * ------------------------------------------------------------------ */
#define TOMCAT_ANGLE_LSB_RAD    (1.0e-4)   /* i32: +/-2.1e5 rad (~34k turns)   */
#define TOMCAT_TENSION_LSB_N    (1.0e-2)   /* u16: 0 .. 655.35 N  (see GAP)    */
#define TOMCAT_CURRENT_LSB_A    (1.0e-3)   /* i16: +/-32.767 A                 */
#define TOMCAT_TEMP_LSB_C       (1.0e-1)   /* i16: +/-3276.7 degC              */
#define TOMCAT_VBUS_LSB_V       (1.0e-2)   /* u16: 0 .. 655.35 V               */
/* Stiffness LSB: agonist AIC gain k, tension per unit CABLE displacement.
 * Units N/m referenced to cable length (tendon.py law T=T_bias+k*(l-l*)).
 * RT converts rotor angle (rad) <-> cable length (m) via spool radius. GAP. */
#define TOMCAT_STIFFNESS_LSB_N_PER_M (1.0e-1) /* u16: 0 .. 6553.5 N/m         */

/* ================================================================== *
 *  Common enums / bitfields (shared by BOTH layers)                   *
 * ================================================================== */

/* Per-motor control mode. Unknown/other -> treat as LIMP (fail safe). */
typedef enum {
    TOMCAT_MODE_LIMP     = 0u, /* de-energize, zero torque, cable slack       */
    TOMCAT_MODE_POSITION = 1u, /* closed-loop rotor-angle (position) tracking */
    TOMCAT_MODE_TENSION  = 2u, /* closed-loop motor-side tension tracking     */
    TOMCAT_MODE_HYBRID   = 3u  /* position + tension/AIC co-contraction (ADR-0002) */
} tomcat_mode_t;

/* Fault-flag bitfield (16-bit). Reported in telemetry at both layers.
 * Any set bit above LIMP_ACTIVE that is NOT explicitly commanded implies the
 * driver has autonomously entered limp (Tier A) or is being held in it. */
typedef enum {
    TOMCAT_FAULT_NONE            = 0x0000u,
    TOMCAT_FAULT_OVERCURRENT     = 0x0001u, /* Tier A latch (driver-local)     */
    TOMCAT_FAULT_OVERTENSION     = 0x0002u, /* Tier A latch                    */
    TOMCAT_FAULT_OVERTEMP        = 0x0004u, /* Tier A latch                    */
    TOMCAT_FAULT_ROTOR_SENSOR    = 0x0008u, /* encoder/Hall loss (ADR-0004)    */
    TOMCAT_FAULT_TENSION_SENSOR  = 0x0010u, /* tension front-end fault         */
    TOMCAT_FAULT_CAN_TIMEOUT     = 0x0020u, /* driver rx-watchdog (Tier C arm) */
    TOMCAT_FAULT_DRIVER_FAULT    = 0x0040u, /* gate driver / stage fault       */
    TOMCAT_FAULT_ESTOP_ACTIVE    = 0x0080u, /* Tier B hardware e-stop asserted */
    TOMCAT_FAULT_LIMP_ACTIVE     = 0x0100u, /* currently in limp state         */
    TOMCAT_FAULT_WATCHDOG_TRIP   = 0x0200u, /* Tier C RT-supervisor watchdog   */
    TOMCAT_FAULT_CAL_INVALID     = 0x0400u, /* no valid tension/Kt cal -> safe */
    TOMCAT_FAULT_BUS_OFF         = 0x0800u, /* CAN controller bus-off          */
    TOMCAT_FAULT_OVERVOLTAGE     = 0x1000u, /* bus over/under-voltage          */
    TOMCAT_FAULT_NOT_HOMED       = 0x2000u, /* rotor position not referenced   */
    TOMCAT_FAULT_SETPOINT_CLAMP  = 0x4000u, /* command exceeded a limit, clamped */
    TOMCAT_FAULT_RESERVED_15     = 0x8000u
} tomcat_fault_t;

/* RT-supervisor coarse state (Layer 1 up + SYNC broadcast). */
typedef enum {
    TOMCAT_RT_BOOT    = 0u,
    TOMCAT_RT_IDLE    = 1u, /* enabled, holding safe, no motion authorized     */
    TOMCAT_RT_ACTIVE  = 2u, /* tracking SBC setpoints                          */
    TOMCAT_RT_LIMP    = 3u, /* all motors commanded limp                       */
    TOMCAT_RT_ESTOP   = 4u, /* Tier B latched; recovery requires operator      */
    TOMCAT_RT_FAULT   = 5u  /* internal fault, failed safe                     */
} tomcat_rt_state_t;

/* Reason codes for a forced-limp / safety broadcast. */
typedef enum {
    TOMCAT_LIMP_REASON_NONE        = 0u,
    TOMCAT_LIMP_REASON_OPERATOR    = 1u, /* commanded e-stop / limp            */
    TOMCAT_LIMP_REASON_ESTOP_HW    = 2u, /* Tier B hardware e-stop             */
    TOMCAT_LIMP_REASON_SBC_STALE   = 3u, /* Tier C: lost SBC heartbeat         */
    TOMCAT_LIMP_REASON_DRIVER_LOST = 4u, /* Tier C: lost a driver heartbeat    */
    TOMCAT_LIMP_REASON_DRIVER_FAULT= 5u  /* Tier A latch propagated up         */
} tomcat_limp_reason_t;

/* ================================================================== *
 *  Logical motor addressing (RT-tier global index)                    *
 *  ~33 motors: 24 leg (4 legs x 3 joints x 2 tendons) + 6 spine       *
 *  (3 sagittal segments x 2) + tail (TBD). The RT tier maps each       *
 *  logical id to a (CAN segment, node) pair (see INTERFACE.md table).  *
 * ================================================================== */
#define TOMCAT_N_LEG_MOTORS    24u
#define TOMCAT_N_SPINE_MOTORS   6u   /* M1 sagittal only; grows with ADR-0006 */
#define TOMCAT_N_TAIL_MOTORS    3u   /* placeholder (ADR-0007), bench-TBD     */
#define TOMCAT_N_MOTORS_MAX     33u  /* array sizing upper bound; verify HW   */

/* Tendon side of an antagonistic pair (matches tendon.py flexor/extensor). */
typedef enum {
    TOMCAT_TENDON_FLEXOR   = 0u,
    TOMCAT_TENDON_EXTENSOR = 1u
} tomcat_tendon_side_t;

/* ================================================================== *
 *  LAYER 1 — SBC <-> RT controller (logical, transport-neutral)       *
 *  Not CAN framed; carried over the SBC<->RT link (shared mem / SPI / *
 *  Ethernet — ADR-0005 follow-up). Arrays indexed by logical motor id.*
 * ================================================================== */

/* Per-motor setpoint (SBC -> RT). Field-for-field with TendonSolution:
 *   tension_target  <- motor_tension_flexor/extensor (MOTOR-side, N)
 *   t_bias          <- TendonSolution.t_bias
 *   position_target <- motor angle from TendonMap.motor_angles(q)
 *   stiffness_k     <- AIC agonist gain (dynamic, firmware-side per ADR-0002) */
typedef struct {
    uint8_t  mode;             /* tomcat_mode_t                                */
    uint8_t  flags;            /* reserved command flags (bit0: enable)        */
    uint16_t stiffness_k;      /* TOMCAT_STIFFNESS_LSB_N_PER_M  (GAP: ref frame)*/
    int32_t  position_target;  /* TOMCAT_ANGLE_LSB_RAD (motor-side rotor angle) */
    uint16_t tension_target;   /* TOMCAT_TENSION_LSB_N (motor-side)  (GAP: cal) */
    uint16_t t_bias;           /* TOMCAT_TENSION_LSB_N (co-contraction floor)   */
} tomcat_l1_motor_cmd_t;

/* Whole-frame SBC -> RT command. seq drives the Tier-C SBC watchdog. */
typedef struct {
    uint32_t seq;              /* monotonic; RT declares SBC stale on gap       */
    uint8_t  schema_major;
    uint8_t  schema_minor;
    uint8_t  request;          /* bit0 arm, bit1 request_limp, bit2 clear_faults*/
    uint8_t  n_motors;
    tomcat_l1_motor_cmd_t motor[TOMCAT_N_MOTORS_MAX];
} tomcat_l1_command_t;

/* Per-motor state (RT -> SBC). */
typedef struct {
    int32_t  rotor_angle;      /* TOMCAT_ANGLE_LSB_RAD                          */
    uint16_t tension_meas;     /* TOMCAT_TENSION_LSB_N  (GAP: cal per ADR-0004) */
    int16_t  current;          /* TOMCAT_CURRENT_LSB_A  (GAP: ->torque via Kt)  */
    int16_t  temperature;      /* TOMCAT_TEMP_LSB_C                             */
    uint16_t fault_flags;      /* tomcat_fault_t                                */
    uint8_t  mode;             /* tomcat_mode_t (actual)                        */
    uint8_t  reserved;
} tomcat_l1_motor_state_t;

/* Whole-frame RT -> SBC telemetry. seq drives the SBC-side liveness check. */
typedef struct {
    uint32_t seq;
    uint32_t rt_time_us;       /* RT loop timestamp (us)                        */
    uint8_t  rt_state;         /* tomcat_rt_state_t                             */
    uint8_t  limp_reason;      /* tomcat_limp_reason_t                          */
    uint8_t  n_motors;
    uint8_t  reserved;
    uint16_t fault_summary;    /* OR of all motor fault_flags                   */
    uint16_t reserved2;
    tomcat_l1_motor_state_t motor[TOMCAT_N_MOTORS_MAX];
} tomcat_l1_telemetry_t;

/* ================================================================== *
 *  LAYER 2 — RT <-> smart drivers over CAN-FD                         *
 *  11-bit ID:  ID = (FUNC << 7) | NODE_ID                             *
 *  FUNC low value = higher CAN arbitration priority.                  *
 *  NODE_ID: 1..126 unique WITHIN a segment; 0x7F = broadcast.         *
 *  Segment separation is physical (one CAN-FD controller per segment).*
 * ================================================================== */

#define TOMCAT_CAN_ID_NODE_BITS   7u
#define TOMCAT_CAN_ID_NODE_MASK   0x07Fu
#define TOMCAT_CAN_ID_MAKE(func, node) \
    ((uint16_t)(((func) << TOMCAT_CAN_ID_NODE_BITS) | ((node) & TOMCAT_CAN_ID_NODE_MASK)))
#define TOMCAT_CAN_ID_FUNC(id)    ((uint8_t)((id) >> TOMCAT_CAN_ID_NODE_BITS))
#define TOMCAT_CAN_ID_NODE(id)    ((uint8_t)((id) & TOMCAT_CAN_ID_NODE_MASK))

#define TOMCAT_NODE_BROADCAST     0x7Fu

/* CAN-FD function codes (4-bit). Ordered by priority (lower = higher). */
typedef enum {
    TOMCAT_FUNC_SAFETY   = 0x0u, /* forced limp / e-stop broadcast (highest)   */
    TOMCAT_FUNC_SYNC     = 0x1u, /* cycle trigger + arm/estop bits             */
    TOMCAT_FUNC_COMMAND  = 0x2u, /* RT -> driver setpoint (per node)           */
    TOMCAT_FUNC_TELEM    = 0x3u, /* driver -> RT telemetry (per node)          */
    TOMCAT_FUNC_CFG_WR   = 0x4u, /* RT -> driver config/limit/cal write        */
    TOMCAT_FUNC_CFG_ACK  = 0x5u  /* driver -> RT config ack / info             */
} tomcat_func_t;

/* --- FUNC_SAFETY broadcast payload (4 bytes). Highest priority frame. --- */
typedef struct {
    uint8_t  reason;           /* tomcat_limp_reason_t                          */
    uint8_t  flags;            /* bit0 limp_all, bit1 latch (require clear)     */
    uint16_t crc;              /* frame CRC (polynomial TBD)                    */
} tomcat_can_safety_t;

/* --- FUNC_SYNC broadcast payload (8 bytes). Time-triggers the cycle. --- */
typedef struct {
    uint32_t cycle;            /* monotonic 1 kHz cycle counter                 */
    uint8_t  global_flags;     /* bit0 armed, bit1 estop_active, bit2 limp_all  */
    uint8_t  rt_state;         /* tomcat_rt_state_t                             */
    uint16_t crc;
} tomcat_can_sync_t;

/* --- FUNC_COMMAND payload (RT -> driver), 16 bytes. --- */
typedef struct {
    uint8_t  seq;              /* echoes SYNC cycle low byte (driver rx-wdog)   */
    uint8_t  mode;             /* tomcat_mode_t                                 */
    uint8_t  flags;            /* bit0 enable, bit1 clear_faults                */
    uint8_t  reserved;
    int32_t  position_target;  /* TOMCAT_ANGLE_LSB_RAD (rotor)                  */
    uint16_t tension_target;   /* TOMCAT_TENSION_LSB_N (motor-side)             */
    uint16_t t_bias;           /* TOMCAT_TENSION_LSB_N                          */
    uint16_t stiffness_k;      /* TOMCAT_STIFFNESS_LSB_N_PER_M                  */
    uint16_t reserved2;
} tomcat_can_command_t;

/* --- FUNC_TELEM payload (driver -> RT), 20 bytes. --- */
typedef struct {
    uint8_t  seq;              /* echoes commanded seq (RT match / staleness)   */
    uint8_t  mode_status;      /* low nibble actual mode, high nibble status    */
    uint16_t fault_flags;      /* tomcat_fault_t                                */
    int32_t  rotor_angle;      /* TOMCAT_ANGLE_LSB_RAD                          */
    uint16_t tension_meas;     /* TOMCAT_TENSION_LSB_N (GAP: cal per ADR-0004)  */
    int16_t  current;          /* TOMCAT_CURRENT_LSB_A (GAP: ->torque via Kt)   */
    int16_t  temperature;      /* TOMCAT_TEMP_LSB_C                             */
    uint16_t vbus;             /* TOMCAT_VBUS_LSB_V                             */
    uint16_t reserved;
} tomcat_can_telem_t;

/* --- FUNC_CFG_WR payload (RT -> driver), low-rate limit/cal push, 8 bytes.
 *     key selects which parameter; value is raw (scale per key). This is where
 *     the OPEN ADR-0004 tension transfer + motor Kt land once fixed. --- */
typedef enum {
    TOMCAT_CFG_TENSION_LIMIT   = 1u, /* Tier A over-tension latch threshold (N) */
    TOMCAT_CFG_CURRENT_LIMIT   = 2u, /* Tier A over-current latch threshold (A) */
    TOMCAT_CFG_TEMP_LIMIT      = 3u, /* Tier A over-temp latch threshold (degC) */
    TOMCAT_CFG_RX_TIMEOUT_US   = 4u, /* driver rx-watchdog window (us)          */
    TOMCAT_CFG_TENSION_SCALE   = 5u, /* GAP ADR-0004: sensor LSB -> N transfer  */
    TOMCAT_CFG_MOTOR_KT        = 6u, /* GAP: torque constant (N.m/A)            */
    TOMCAT_CFG_POS_MIN         = 7u,
    TOMCAT_CFG_POS_MAX         = 8u
} tomcat_cfg_key_t;

typedef struct {
    uint8_t  key;              /* tomcat_cfg_key_t                              */
    uint8_t  flags;            /* bit0 persist                                  */
    uint16_t reserved;
    int32_t  value;            /* raw, scale defined per key                    */
} tomcat_can_cfg_t;

/* ------------------------------------------------------------------ *
 *  Wire-size documentation (compile-time; not runtime logic).         *
 *  These pin the byte budget the INTERFACE.md bus-load calc assumes.  *
 * ------------------------------------------------------------------ */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(tomcat_can_safety_t)  == 4,  "safety frame = 4 B");
_Static_assert(sizeof(tomcat_can_sync_t)    == 8,  "sync frame = 8 B");
_Static_assert(sizeof(tomcat_can_command_t) == 16, "command frame = 16 B");
_Static_assert(sizeof(tomcat_can_telem_t)   == 20, "telem frame = 20 B");
_Static_assert(sizeof(tomcat_can_cfg_t)     == 8,  "cfg frame = 8 B");
#endif
/* NOTE: no packing pragma is applied; all fields above are laid out to be
 * naturally aligned so sizeof matches the wire byte count without padding.
 * Serializers must still write little-endian regardless of host order. */

#ifdef __cplusplus
}
#endif
#endif /* TOMCAT_CAN_SCHEMA_H */
