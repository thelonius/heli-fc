#ifndef SFHSS_H
#define SFHSS_H

#include <stdint.h>

/*
 * Futaba S-FHSS receiver, ported from opiopan/rcstick-f (firmware/Src/sfhss.c).
 * 30 hop channels, one CC2500 channel step = 6*n+16, 13-byte fixed packets
 * at 128kbps 2-FSK, sync word 0xD391, packet pair (ch1-4 / ch5-8) every ~6.8ms.
 * Channel values arrive as servo pulse width in microseconds, center 1520.
 */

#define SFHSS_CHNUM   30
#define SFHSS_CENTER  1520

typedef enum {
    SFHSS_PH_START_BIND = 0,
    SFHSS_PH_FINDING,      /* waiting for the first packet on hop channel 0 */
    SFHSS_PH_BINDED,       /* (re)entering connection on hop channel 0 */
    SFHSS_PH_CONNECTED     /* hopping in sync with the transmitter */
} SFHSS_Phase_t;

typedef struct {
    SFHSS_Phase_t phase;
    uint8_t  ch;                 /* hop index 0..29 */
    uint8_t  hopcode;            /* from packet, defines the hop sequence */
    uint8_t  nextSlot;           /* frame we're waiting for: 0=ch1-4, 1=ch5-8 */
    int32_t  txaddr;             /* bound transmitter id, -1 = none */
    uint32_t rtime;              /* last GDO2 assert time, us (24-bit) */
    uint32_t lastFrameTime;      /* time of the last successfully decoded frame */
    uint32_t lastFrameSystick;    /* Systick value of last frame, for failsafe */
    uint32_t interval;           /* expected time between frames, us; fixed at
                                    the protocol's ~6801us, never measured */
    uint16_t skipCount;          /* consecutive blind hops */
    uint16_t data[8];            /* channel values, usec, center 1520 */
    uint8_t  rssi;               /* raw RSSI byte from the last packet */
    int8_t   freqOffset;         /* FSCTRL0 value found during binding */
    uint8_t  caldata[SFHSS_CHNUM]; /* FSCAL1 per hop channel */
    /* debug counters, read via SWD */
    uint32_t statRcv;
    uint32_t statLost;
    uint32_t statBadPkt;
    uint32_t statRebind;
    uint32_t statFailsafe;
    uint32_t statAttempts;    /* RXBYTES crossed threshold, FIFO read attempted */
    uint32_t statNoMarker;    /* pkt[0] != 0x81 */
    uint32_t statNoCrc;       /* CRC_OK bit missing */
    uint32_t statOverflow;    /* RX FIFO overflow seen */
    uint8_t  lastpkt[15];     /* raw bytes of the last FIFO read, any outcome */
    uint8_t  lastRxbytes;     /* last RXBYTES register value polled */
    uint8_t  lastFifoStatus;  /* last FIFO status byte from read attempt */
    /* parity diagnostics (keep at the end — SWD scripts rely on the offsets
     * of the fields above): how many valid frames carried cmd&1==0 (ch1-4)
     * vs cmd&1==1 (ch5-8). Settles whether ch5-8 frames arrive at all when
     * hopping on every frame. */
    uint32_t cntEven;
    uint32_t cntOdd;
    /* last-16-frames ring: time since the previous valid frame (us), the raw
     * 6-bit cmd, and the hop index it arrived on. Read over SWD to learn the
     * TX's true frame/parity/hop timing instead of guessing it. */
    uint32_t ringDelta[16];
    uint8_t  ringCmd[16];
    uint8_t  ringCh[16];
    uint8_t  ringIdx;
    uint32_t cntHarvest;      /* harvest attempts (trailer lingers) started */
    uint8_t  sigLost;         /* latched signal-loss flag: set by SFHSS_Poll
                                 0.5s after the last valid frame, cleared ONLY
                                 by an actually received frame. HasSignal()
                                 returns !sigLost — a bare elapsed() compare
                                 there goes false-positive for 0.5s every
                                 2^24us (16.8s) SysTick wrap, which made the
                                 servos jerk out of failsafe every ~15s. */
    /* Snapshot of the last ODD (ch5-8) frame, stored even when the frame is
     * failsafe-flagged (cmd&4) and its data[] update is suppressed. This TX
     * sends odd frames so rarely (failsafe broadcasts ~1/30s + bursts on
     * setting changes) that lastpkt — overwritten every 6.8ms by even frames —
     * can never be caught holding one over SWD. Diagnostic for learning what
     * those frames actually carry. */
    uint16_t oddData[4];      /* decoded d[0..3] of the last ch5-8 frame */
    uint8_t  oddCmd;          /* its raw 6-bit cmd */
    /* Histogram of the frame TYPE field (cmd bits1-5, indexed &15). Kept last
     * so it never shifts the offsets the SWD scripts hardcode. Reveals which
     * types the TX actually streams — the key to whether ch5-8/collective is
     * fast (stock accepts types 4,5,7 as ch5-8). */
    uint32_t typeCount[16];
    /* SLOT GRID anchor (us): the expected prompt-read time of the current
     * slot's DATA1. Re-anchored on every received frame (with a -1625us
     * correction when the read also contained DATA2, i.e. happened late);
     * advanced by exactly `interval` on every hop — received or not — so
     * blind hops stay phase-aligned with the TX instead of chasing it.
     * May point slightly into the future right after a hop. Kept last-ish so
     * it never shifts the offsets SWD scripts hardcode. */
    uint32_t slotStart;
    /* flywheel diagnostics (appended 2026-07-12; re-derive SWD offsets from
     * DWARF after any struct change — see tools/read_sfhss.sh header) */
    uint32_t statResync;    /* CONNECTED -> rebind fallbacks (desync episodes) */
    uint32_t maxPollGap;    /* max us between consecutive SFHSS_Poll entries;
                               instruments the both-packets-in-one-poll anomaly.
                               Reset over SWD (mww) to re-arm. */
    uint32_t lastPollTime;  /* previous SFHSS_Poll entry time, us */
    uint8_t  pollSeen;      /* lastPollTime holds a real sample */
    uint8_t  frameThisSlot; /* >=1 valid frame received in the current slot */
    /* Cold-start freeze diagnostics (2026-07-23). During the connect-time
     * freeze only hop channels 0-3 decoded (SWD ring dump) — reception is
     * FREQUENCY-dependent. These localize it: per-channel tallies plus the
     * chip's FREQEST offset estimate (1 LSB = 26MHz/2^14 ~ 1.59 kHz), read
     * once per FIFO drain. Passive: nothing feeds back into tuning.
     * Offsets (g_sfhss+): rcvByCh 0x152, badByCh 0x18e, festByCh 0x1ca,
     * festBadByCh 0x1e8, ringFest 0x206, lastFest 0x216. */
    uint16_t rcvByCh[SFHSS_CHNUM];     /* valid frames per hop index */
    uint16_t badByCh[SFHSS_CHNUM];     /* marker OK but CRC fail, per hop index */
    int8_t   festByCh[SFHSS_CHNUM];    /* FREQEST of the last valid frame */
    int8_t   festBadByCh[SFHSS_CHNUM]; /* FREQEST of the last CRC-fail frame */
    int8_t   ringFest[16];             /* FREQEST alongside ringDelta/Cmd/Ch */
    int8_t   lastFest;                 /* FREQEST of the last drain, any outcome */
    /* S1 sync-ritual diagnostics (2026-07-23, docs/sync_ritual.md):
     * statAnchorRej +0x218, statHold +0x21a. */
    uint16_t statAnchorRej;   /* frames whose drain time was too late to anchor */
    uint16_t statHold;        /* miss-ladder park entries (grid held, radio parked) */
    /* S2 measured-interval DIAGNOSTIC (does not drive `interval`; see sfhss.c).
     * intervalMeas = median slot period from prompt DATA1-only reads, us;
     * 0 = not enough clean samples yet. Read over SWD to watch for TX drift or
     * a foreign TX. Re-derive offsets from DWARF after any change. */
    uint16_t intervalMeas;    /* measured slot period, us (0 = unmeasured) */
    uint8_t  syncCount;       /* accepted samples so far (caps at 16) */
    uint8_t  lastData1Ok;     /* lastData1 holds a real prompt-DATA1 timestamp */
    uint32_t lastData1;       /* rtime of the last prompt DATA1, for the delta */
    uint16_t syncSamp[16];    /* accepted DATA1->DATA1 deltas, us */
} SFHSS_t;

extern SFHSS_t g_sfhss;
extern uint8_t g_cc2500_partnum;  /* expect 0x80 */
extern uint8_t g_cc2500_version;  /* expect 0x03 */
extern uint8_t g_cc2500_init_tries; /* attempts the last SFHSS_Init needed */
extern uint8_t g_cc2500_cfg_ok;     /* 1 = config readback verified */
extern uint8_t g_cc2500_cfg_rb[0x27];

/* Bench knob, poked over SWD (mwb) during a caught connect-freeze to bisect
 * the mechanism live. Idle = 0 = zero behavior change. Handled at the top of
 * SFHSS_Poll, self-clears when done:
 *   1 = full radio re-init (SRES+config+30ch cal; WIPES g_sfhss diagnostics —
 *       dump them first)
 *   2 = stock-style per-hop autocal (MCSM0 FS_AUTOCAL=01, +~721us per hop)
 *   3 = back to boot-cal + manual FSCAL1 restore (MCSM0 autocal off)
 *   4 = pause IMU I2C traffic (main loop skips sampling)   5 = resume it */
extern volatile uint8_t g_radio_cmd;
extern volatile uint8_t g_imu_pause;

void SFHSS_Init(void);            /* reset chip, load config, calibrate 30 channels */
void SFHSS_Poll(uint32_t now_us); /* call from the main loop as often as possible */
uint8_t SFHSS_Connected(void);        /* instantaneous: phase == CONNECTED (flickers) */
uint8_t SFHSS_HasSignal(uint32_t now_us); /* stable: a valid frame within the last 0.5s */
uint8_t SFHSS_QuietForImu(uint32_t now_us); /* slot dead zone: safe to stall ~1.5ms (IMU read) */

#endif /* SFHSS_H */
