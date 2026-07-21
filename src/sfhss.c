#include "sfhss.h"
#include "spi_rc.h"

/* Pinned to 0x20000000 by the linker (.sfhss_state); NOLOAD, so zeroed in Init. */
SFHSS_t __attribute__((section(".sfhss_state"))) g_sfhss;
uint8_t g_cc2500_partnum;
uint8_t g_cc2500_version;

/* micros() returns real us but wraps every 1,000,000 us (1s) — see main.c
 * clock_init. All time math here is modulo TIME_WRAP. */
#define TIME_WRAP 1000000u
#define TIME_MASK 0x00FFFFFFu   /* legacy no-op mask; micros() values are < 1e6 */

#define SHORT_INTERVAL      6801u  /* protocol constant: inter-frame period, us */
#define HOPPING_TIMEOUT     500u   /* us of slack past the expected packet */
/* Blind grid-hops before falling back to rebind. Flywheel hops stay
 * phase-aligned (crystal drift <=80ppm = ~35us over 64 slots, well inside the
 * window), so riding a long fade costs nothing and any caught frame re-anchors
 * instantly; rebinding instead means a ~200ms camp on channel 0. 64 slots
 * ~435ms sits just under the 500ms failsafe latch: by the time we give up and
 * rebind, the outputs are already in failsafe anyway. */
#define FALLBACK_COUNT      64

#define PACKET_LENGTH 15           /* 13 data bytes + RSSI + LQI/CRC_OK */

/* Sustained-loss threshold: no valid frame for this long latches sigLost, which
 * drops SFHSS_HasSignal() and sends the outputs to failsafe (swash centre, motor
 * off). Long enough to ride through normal hop gaps + a burst of missed slots
 * (each slot is ~6.8ms), short enough to centre the swash promptly when the TX
 * really goes away. Standard S-FHSS failsafe delay is ~0.5-1s. */
#define SIGNAL_LOSS_US 500000u

/* Receive strategy. Mode 0 is production: since 2026-07-07 it implements the
 * STOCK discipline decoded from the original firmware (hop after the slot's
 * parity-1 packet, or SLOT_2ND_WINDOW_US after the parity-0 packet) — this is
 * what finally receives ch5-8/collective, which the TX streams ~10/s in
 * idle-up (data[4]=1210 <1520). Modes 1-2 are RETIRED experiments (their
 * CONNECTED-path code was removed with the 2026-07-07 rewrite; leftovers in
 * FINDING/miss_frame are inert). Mode 3 = scanner, mode 4 = harvest, both
 * diagnostic. Historical notes:
 *   0 (pre-2026-07-07) = hop on every valid frame: ~90/s ch1-4 frames but
 *       cntOdd == 0 over 30s — the ch5-8 frame is sent mid-slot on the SAME
 *       channel after the ch1-4 frame, and hopping right away always misses it.
 *   1 = 7aadfec "pair" model: waits for the odd frame but on a miss hops at
 *       interval+HOPPING_TIMEOUT — 500us AFTER the next even frame lands on
 *       the next channel, so it chronically chases the TX (~2/s, rebind churn).
 *   2 = linger: stay on the channel after an even frame to catch the odd one,
 *       but hop unconditionally at LINGER_US (< 6.8ms slot), so a missed odd
 *       frame costs nothing and there is no chase. Measured: cntOdd still 0 —
 *       no odd frame arrives on the same channel within 5.5ms of the even.
 *   3 = SCANNER (diagnostic only): park on one channel forever and ring-log
 *       everything that lands there, to learn the TX's real per-channel
 *       schedule (parity, intra-visit gaps, revisit period). Findings: each
 *       channel gets its main even frame every ~204ms (30 slots), plus a
 *       TRAILING frame ~13.6ms (even) or ~15.1ms (odd, cmd=0x0b) after it,
 *       alternating between visits. That's where ch5-8 lives — ~2 slots
 *       behind the main sequence, which is why modes 0-2 never saw it.
 *   4 = HARVEST: hop on every frame like mode 0, but every HARVEST_EVERY-th
 *       catch stay on the channel up to HARVEST_WINDOW_US for the trailing
 *       frame (odd half the time -> ch5-8 update), then rejoin the sequence
 *       3 channels ahead (2 slots were spent waiting). Costs ~25% of the
 *       even rate, buys periodic collective/ch5-8 data. */
#define SFHSS_PAIR_MODE 0
#define HARVEST_EVERY      6      /* linger for the trailer every N catches */
#define HARVEST_WINDOW_US  16500u /* covers the +13.6/+15.1ms trailer arrivals */
#define LINGER_US 5500u  /* max wait for the ch5-8 frame; well short of the
                            6801us slot so the next-channel retune wins the race */
/* Max linger after DATA1 before hopping, us. The TX sends DATA2 exactly
 * SFHSS_DATA2_TIMING=1625us after DATA1 (start-to-start, per DIY-Multiprotocol
 * Futaba_cc2500.ino). We read DATA1 ~1310us into its airtime, so DATA2 becomes
 * readable ~1625us after lastFrameTime; margin here covers poll latency. The TX
 * doesn't hop until ~4800us after DATA1, so a wider window is safe. */
#define SLOT_2ND_WINDOW_US 2100u
/* DATA2 trails DATA1 by this much start-to-start (and, at equal airtime, also
 * read-to-read). Used to correct the grid anchor when a drain contained DATA2:
 * such a read happened at/after DATA2-complete, i.e. this much later than a
 * prompt DATA1 read would have. */
#define SLOT_DATA2_OFFSET_US 1625u

/* CC2500 config for S-FHSS (registers 0x00..0x26) — EXACT rcstick-f values,
 * a known-working S-FHSS receiver. Do not "improve" these: the earlier wide
 * filter (MDMCFG4=0x2C) triggered continuously on noise and kept the RX FIFO
 * permanently overflowed. MDMCFG4=0x7C = 232kHz filter BW, MDMCFG2=0x83. */
static const uint8_t sfhss_config[0x27] = {
    /* 00 */ 0x07, 0x2E, 0x3F, 0x07, 0xD3, 0x91, 0x0D, 0x04,
    /* 08 */ 0x0C, 0x29, 0x10, 0x06, 0x00, 0x5C, 0x4E, 0xC4,
    /* 10 */ 0x7C, 0x43, 0x83, 0x23, 0x3B, 0x44, 0x07, 0x3C,
    /* 18 */ 0x08, 0x1D, 0x1C, 0x43, 0x40, 0x91, 0x57, 0x6B,
    /* 20 */ 0xF8, 0xB6, 0x10, 0xEA, 0x0A, 0x11, 0x11
};

static uint8_t pktbuf[1 + PACKET_LENGTH]; /* [0] = chip status byte */

#if SFHSS_PAIR_MODE == 4
static uint8_t  harvest_active;  /* lingering for the trailing frame */
static uint8_t  harvest_count;   /* catches since the last harvest attempt */
static uint32_t harvest_t0;      /* rtime of the anchor frame */
#endif

static uint32_t elapsed(uint32_t since, uint32_t now) {
    int32_t d = (int32_t)now - (int32_t)since;
    if (d < 0) d += TIME_WRAP;
    return (uint32_t)d;
}

/* Signed time since `t`, for timestamps that may sit slightly in the FUTURE
 * (the slot-grid anchor right after a hop). elapsed() would alias a future
 * t into a huge positive value and fire the hop window instantly; here a
 * small negative difference stays negative, and only a gap wider than half
 * the wrap is treated as a wraparound. */
static int32_t since_signed(uint32_t t, uint32_t now) {
    int32_t d = (int32_t)now - (int32_t)t;
    if (d < -(int32_t)(TIME_WRAP / 2)) d += TIME_WRAP;
    /* Fold the other direction too (2026-07-17). A FUTURE t that has already
     * wrapped while now hasn't (slotStart straddling the 1s micros() rollover)
     * came out as ~+1e6 instead of small-negative, so the hop window fired
     * instantly and machine-gunned blind hops until now wrapped as well —
     * measured as 1-6 lost slots once per second, walking through the slot
     * phase at 253us/s (1e6 mod 6801). */
    else if (d > (int32_t)(TIME_WRAP / 2)) d -= TIME_WRAP;
    return d;
}

static uint8_t hop_channr(void) {
    return (uint8_t)(g_sfhss.ch * 6 + 16);
}

/* Retune: manual FSCAL1 because autocalibration is off (MCSM0=0x08) */
static void hop(void) {
    CC2500_Strobe(CC2500_SIDLE);
    CC2500_WriteReg(CC2500_FSCAL1, g_sfhss.caldata[g_sfhss.ch]);
    CC2500_WriteReg(CC2500_CHANNR, hop_channr());
    CC2500_Strobe(CC2500_SRX);
}

static void hop_flush(void) {
    CC2500_Strobe(CC2500_SIDLE);
    CC2500_WriteReg(CC2500_FSCAL1, g_sfhss.caldata[g_sfhss.ch]);
    CC2500_WriteReg(CC2500_CHANNR, hop_channr());
    CC2500_Strobe(CC2500_SFRX);
    CC2500_Strobe(CC2500_SRX);
}

/* Flush garbage (CRC fail / overflow) without changing the channel */
static void rx_restart(void) {
    CC2500_Strobe(CC2500_SIDLE);
    CC2500_Strobe(CC2500_SFRX);
    CC2500_Strobe(CC2500_SRX);
}

static void next_channel(void) {
    uint8_t ch = g_sfhss.ch + g_sfhss.hopcode + 2;
    if (ch > 29) {
        if (ch < 31) ch += g_sfhss.hopcode + 2;
        ch -= 31;
    }
    g_sfhss.ch = ch;
}

/* Drain one fixed-length frame from the FIFO. Overflow is handled by the
 * caller before we get here, so this just reads and records for debugging. */
static void read_packet(void) {
    g_sfhss.statAttempts++;
    uint8_t status = CC2500_ReadFIFO(&pktbuf[1], PACKET_LENGTH);
    pktbuf[0] = status;
    g_sfhss.lastFifoStatus = status;
    for (int i = 0; i < PACKET_LENGTH; i++) g_sfhss.lastpkt[i] = pktbuf[1 + i];
}

static uint8_t parse_packet(uint8_t *cmd) {
    /* The chip-status byte reads unreliably here (0xFF under overflow), so
     * don't gate on it — the frame marker and CRC_OK bit fully validate. */
    uint8_t *pkt = &pktbuf[1];
    if (pkt[0] != 0x81) { g_sfhss.statNoMarker++; return 0; } /* S-FHSS frame marker */
    if (!(pkt[14] & 0x80)) { g_sfhss.statNoCrc++; return 0; } /* appended LQI: CRC_OK */

    int32_t txaddr = ((int32_t)pkt[1] << 8) | pkt[2];
    uint8_t hopcode = (uint8_t)(((pkt[11] & 0x07) << 2) | ((pkt[12] & 0xC0) >> 6));
    *cmd = pkt[12] & 0x3F;

    if (g_sfhss.txaddr < 0) g_sfhss.txaddr = txaddr;
    if (g_sfhss.txaddr != txaddr) {
        g_sfhss.statBadPkt++;
        return 0;
    }
    g_sfhss.hopcode = hopcode;
    g_sfhss.rssi = pkt[13];

    uint16_t d[4];
    d[0] = (uint16_t)(((pkt[5] & 0x07) << 9) | (pkt[6] << 1) | ((pkt[7] & 0x80) >> 7));
    d[1] = (uint16_t)(((pkt[7] & 0x7F) << 5) | ((pkt[8] & 0xF8) >> 3));
    d[2] = (uint16_t)(((pkt[8] & 0x07) << 9) | (pkt[9] << 1) | ((pkt[10] & 0x80) >> 7));
    d[3] = (uint16_t)(((pkt[10] & 0x7F) << 5) | ((pkt[11] & 0xF8) >> 3));

    /* cmd = pkt[12] & 0x3F. AUTHORITATIVE bit meaning from DIY-Multiprotocol
     * Futaba_cc2500.ino (see memory heli-sfhss-true-protocol):
     *   bit0 (0x01) = packet number: 0 = DATA1 (first of hop), 1 = DATA2.
     *   bit2 (0x04) = failsafe transmission (not live stick data).
     *   bit3 (0x08) = channel bank: 0 = CH1-4, 1 = CH5-8.
     * The earlier "type = (cmd>>1)&0x1F" model was wrong — the bank is a single
     * bit (bit3), not a 5-bit type. */
    g_sfhss.typeCount[*cmd & 15]++;   /* histogram of the low cmd nibble */

    if (*cmd & 0x04) {
        g_sfhss.statFailsafe++;       /* failsafe frame — not live sticks */
    } else if (*cmd & 0x08) {
        for (int i = 0; i < 4; i++) g_sfhss.data[4 + i] = d[i];  /* CH5-8 */
        for (int i = 0; i < 4; i++) g_sfhss.oddData[i] = d[i];   /* SWD snapshot */
        g_sfhss.oddCmd = *cmd;
    } else {
        for (int i = 0; i < 4; i++) g_sfhss.data[i] = d[i];      /* CH1-4 */
    }

    /* timing diagnostic: delta since the previous valid frame + cmd + hop ch */
    uint8_t ri = g_sfhss.ringIdx & 15;
    g_sfhss.ringDelta[ri] = elapsed(g_sfhss.lastFrameTime, g_sfhss.rtime);
    g_sfhss.ringCmd[ri]   = *cmd;
    g_sfhss.ringCh[ri]    = g_sfhss.ch;
    g_sfhss.ringIdx++;

    g_sfhss.lastFrameTime = g_sfhss.rtime;
    g_sfhss.sigLost = 0;   /* only a real frame revives HasSignal() */

    return 1;
}

#if SFHSS_PAIR_MODE == 4
/* RETIRED for mode 0 (2026-07-12): the flywheel grid hop replaced this. Kept
 * for the mode-4 harvest diagnostic, which still hops reactively.
 * The expected frame never arrived. The TX leaves a channel only after the
 * odd (ch5-8) frame, so hop only when that is the frame we missed; after a
 * missed even frame the odd one is still due on the current channel. */
static void miss_frame(void) {
    g_sfhss.skipCount++;
    if (g_sfhss.skipCount >= FALLBACK_COUNT) {
        g_sfhss.phase = SFHSS_PH_BINDED; /* keep txaddr/hopcode, reconnect */
        return;
    }
    g_sfhss.statLost++;
#if SFHSS_PAIR_MODE == 1
    if (g_sfhss.nextSlot == 1) {
        next_channel();
        hop_flush();
        g_sfhss.nextSlot = 0;
    } else {
        rx_restart();
        g_sfhss.nextSlot = 1;
    }
#else
    next_channel();
    hop_flush();
    g_sfhss.nextSlot = 0;
#endif
}
#endif /* SFHSS_PAIR_MODE == 4 */

static void start_bind(uint32_t now) {
    g_sfhss.txaddr = -1;
    g_sfhss.interval = SHORT_INTERVAL;
    g_sfhss.skipCount = 0;
    g_sfhss.nextSlot = 0;

    CC2500_Strobe(CC2500_SIDLE);
    CC2500_Strobe(CC2500_SFRX);
    g_sfhss.ch = 0;
    hop();

    g_sfhss.rtime = now & TIME_MASK;
    g_sfhss.phase = SFHSS_PH_FINDING;
}

void SFHSS_Init(void) {
    /* g_sfhss lives in NOLOAD RAM; clear it here (bss loop doesn't reach it) */
    uint8_t *p = (uint8_t *)&g_sfhss;
    for (unsigned i = 0; i < sizeof(g_sfhss); i++) p[i] = 0;

    for (int i = 0; i < 8; i++) g_sfhss.data[i] = SFHSS_CENTER;
    g_sfhss.data[2] = 880; /* throttle low until first frame */
    g_sfhss.txaddr = -1;

    CC2500_Reset();
    for (volatile uint32_t d = 0; d < 64000; d++);  /* ~8ms at 8MHz */
    g_cc2500_partnum = CC2500_ReadStatusReg(CC2500_PARTNUM);
    g_cc2500_version = CC2500_ReadStatusReg(CC2500_VERSION);

    CC2500_WriteBurst(0x00, sfhss_config, sizeof(sfhss_config));

    /* Per-channel synthesizer calibration (autocal is disabled) */
    for (uint8_t i = 0; i < SFHSS_CHNUM; i++) {
        IWDG_KR = IWDG_REFRESH;
        CC2500_Strobe(CC2500_SIDLE);
        CC2500_WriteReg(CC2500_CHANNR, (uint8_t)(i * 6 + 16));
        CC2500_Strobe(CC2500_SCAL);
        CC2500_WaitState(CC2500_STATE_IDLE);
        g_sfhss.caldata[i] = CC2500_ReadReg(CC2500_FSCAL1);
    }

    g_sfhss.phase = SFHSS_PH_START_BIND;
}

uint8_t SFHSS_Connected(void) {
    return g_sfhss.phase == SFHSS_PH_CONNECTED;
}

/* True while a valid frame has arrived recently. Unlike SFHSS_Connected(),
 * this does NOT flicker with the per-hop phase changes — it stays true across
 * the brief re-acquisition gaps that happen constantly during hopping, and
 * only drops on a *sustained* signal loss. This is what the servo/ESC outputs
 * should gate on, so they hold the last stick values instead of chattering
 * between live data and the failsafe floor. */
/* True when the radio is in the DEAD ZONE of the current slot — just hopped,
 * next DATA1 not due for >=~1.6ms — so the caller may stall the loop with a
 * long atom (the IMU I2C burst) without risking packet/hop timing. After a
 * grid hop slotStart points at the NEXT expected DATA1 read, i.e. the future:
 * since_signed() is negative, and anything earlier than -1600us is quiet. Not
 * CONNECTED = nothing to protect (acquisition camps on a channel for ~200ms;
 * a 1ms stall there is noise). */
uint8_t SFHSS_QuietForImu(uint32_t now) {
    if (g_sfhss.phase != SFHSS_PH_CONNECTED) return 1;
    int32_t d = since_signed(g_sfhss.slotStart, now & TIME_MASK);
    return d < -1600;
}

uint8_t SFHSS_HasSignal(uint32_t now) {
    /* Do NOT gate on phase == CONNECTED: the phase legitimately flickers
     * FINDING<->CONNECTED every hop cycle, and cutting the outputs on each
     * flicker makes the servos/ESC chatter against the failsafe floor (the
     * "gives throttle+pitch then off, repeatedly" symptom).
     *
     * Also do NOT return a bare elapsed() compare: elapsed() is modulo 2^24us,
     * so with the TX off (lastFrameTime frozen) it swings back under the 0.5s
     * threshold for 0.5s once every 16.8s SysTick wrap — the outputs left
     * failsafe and jerked stale stick values every ~15s. The loss condition is
     * therefore LATCHED in SFHSS_Poll (which runs every loop, so it observes
     * the timeout long before the wrap) and cleared only by a real frame. */
    (void)now;
    return g_sfhss.txaddr >= 0 && !g_sfhss.sigLost;
}

void SFHSS_Poll(uint32_t now) {
    now &= TIME_MASK;

    /* Poll-cadence instrumentation: the bench shows DATA1+DATA2 almost always
     * drained by ONE poll, which requires a >=1.6ms gap between polls — yet no
     * code path obviously blocks that long. Record the worst gap so the theory
     * is measurable over SWD instead of argued about. */
    if (g_sfhss.pollSeen) {
        uint32_t gap = elapsed(g_sfhss.lastPollTime, now);
        if (gap > g_sfhss.maxPollGap) g_sfhss.maxPollGap = gap;
    }
    g_sfhss.lastPollTime = now;
    g_sfhss.pollSeen = 1;

    /* Latch signal loss here: Poll runs every main-loop pass (~100us), so it
     * sees elapsed() cross 0.5s within one pass of the true timeout — long
     * before the 16.8s wrap can fold elapsed() back under the threshold. Once
     * set, only parse_packet() (a real frame) clears it. */
    if (g_sfhss.txaddr >= 0 &&
        elapsed(g_sfhss.lastFrameTime, now) >= SIGNAL_LOSS_US) {
        g_sfhss.sigLost = 1;
    }

    /* Read RXBYTES (bit7=overflow, bits6-0=count) and, if a full frame is
     * sitting in the FIFO, drain and decode it *this* poll. Deciding on the
     * spot — instead of flagging "received" and reading next pass — avoids the
     * overflow deadlock that discarded every packet before.
     * res: 0 = nothing, 1 = valid frame, 2 = read but rejected. */
    uint8_t rxbytes = CC2500_ReadRxBytes();
    g_sfhss.lastRxbytes = rxbytes;
    uint8_t res = 0, cmd = 0, got_data1 = 0, got_data2 = 0;
    uint8_t overflow = rxbytes & 0x80;
    uint8_t nbytes = rxbytes & 0x7F;

    /* DRAIN ALL COMPLETE PACKETS this poll, not just one. The chip keeps RX on
     * after a packet (MCSM1 RXOFF_MODE=stay), so BOTH slot packets — DATA1 and
     * DATA2 (which lands 1625us later, same channel) — sit in the 64-byte FIFO
     * by the time we poll. The old code read only ONE packet then flushed the
     * rest on overflow, throwing DATA2 (ch5-8) away every hop — that was the
     * "receiver drops the second packet" bug. Loop while a full frame remains;
     * parse each, remembering the last cmd for the hop decision below. */
    if (overflow || nbytes >= PACKET_LENGTH) {
        g_sfhss.rtime = now;
        /* Guard = 2: a slot carries at most DATA1+DATA2. Anything beyond two
         * packets is junk, and the overflow branch below flushes it anyway —
         * while each extra FIFO read is ~400us of bit-bang SPI stalling the
         * loop (drain-stall begets overflow begets bigger drain). */
        for (int guard = 0; guard < 2; guard++) {
            read_packet();
            if (parse_packet(&cmd)) {
                res = 1;
                /* per-PACKET parity counters (they used to count once per
                 * poll off the last cmd, hiding DATA1 when both packets were
                 * drained together — the even=2.6/s artifact) */
                if ((cmd & 1) == 0) { got_data1 = 1; g_sfhss.cntEven++; }
                else                { got_data2 = 1; g_sfhss.cntOdd++;  }
            } else if (res == 0) res = 2;
            rxbytes = CC2500_ReadRxBytes();
            if ((rxbytes & 0x7F) < PACKET_LENGTH) break;
        }
        if (overflow) {
            /* Overflow parks the chip in RXFIFO_OVERFLOW — it receives NOTHING
             * until SFRX, so a restart here is mandatory. */
            g_sfhss.statOverflow++;
            rx_restart();
        }
        /* No overflow: leave RX RUNNING. The old unconditional restart here
         * strobed SIDLE+SFRX right after draining DATA1 — while DATA2 could be
         * airing into the FIFO — killing it mid-packet. Residual junk is
         * bounded: every hop is hop_flush(), and a rejected read below
         * (res==2) still restarts. */
    }

    switch (g_sfhss.phase) {

    case SFHSS_PH_START_BIND:
        start_bind(now);
        break;

    case SFHSS_PH_FINDING:
        /* First valid frame gives txaddr+hopcode (captured in parse_packet,
         * CRC-verified) — enough to start following the hop sequence. */
        if (res == 1) {
            g_sfhss.skipCount = 0;
#if SFHSS_PAIR_MODE == 1 || SFHSS_PAIR_MODE == 2
            if (cmd & 1) {              /* pair complete → TX hops now */
                next_channel();
                hop();
                g_sfhss.nextSlot = 0;
            } else {                    /* ch5-8 frame still due here */
                CC2500_Strobe(CC2500_SRX);
                g_sfhss.nextSlot = 1;
            }
#else
            next_channel();
            hop();
            /* Anchor the slot grid: the DATA1 of the channel we just hopped
             * to is expected one interval after THIS slot's DATA1 (whose
             * prompt-read time is rtime, minus the DATA2 trail if this drain
             * contained DATA2 — i.e. was read late). */
            {
                uint32_t anchor = (got_data1 && !got_data2)
                    ? g_sfhss.rtime
                    : (g_sfhss.rtime + TIME_WRAP - SLOT_DATA2_OFFSET_US) % TIME_WRAP;
                g_sfhss.slotStart = (anchor + g_sfhss.interval) % TIME_WRAP;
            }
            g_sfhss.frameThisSlot = 0;
#endif
            g_sfhss.phase = SFHSS_PH_CONNECTED;
        } else if (res == 2) {
            rx_restart();
        }
        break;

    case SFHSS_PH_BINDED:
        CC2500_Strobe(CC2500_SIDLE);
        CC2500_Strobe(CC2500_SFRX);
        g_sfhss.ch = 0;
        hop();
        g_sfhss.skipCount = 0;
        g_sfhss.phase = SFHSS_PH_FINDING;
        break;

    case SFHSS_PH_CONNECTED:
        /* TRUE slot structure (decoded 2026-07-07 from the original firmware,
         * TIM17 ARR=3400/CNT preloads — see memory heli-sfhss-true-protocol):
         * each ~6.8ms slot carries a first packet (parity 0, ch1-4) and
         * OPTIONALLY a second packet ~1.64ms later (parity 1; ch5-8 when the
         * TX has them — it streams them ~10/s in idle-up, rarely otherwise),
         * after which the TX hops. Stock discipline, replicated below: hop
         * after the parity-1 packet, or SLOT_2ND_WINDOW_US after the parity-0
         * packet if no second one arrives. MCSM1 RXOFF_MODE=11 keeps the chip
         * in RX after a packet, so waiting costs nothing. */
#if SFHSS_PAIR_MODE == 3
        /* Scanner: never hop, never rebind — just log what this channel sees. */
        if (res == 1) {
            g_sfhss.statRcv++;
            if (cmd & 1) g_sfhss.cntOdd++; else g_sfhss.cntEven++;
            rx_restart();
        } else if (res == 2) {
            rx_restart();
        }
        break;
#endif
#if SFHSS_PAIR_MODE == 4
        if (res == 1) {
            g_sfhss.statRcv++;
            g_sfhss.skipCount = 0;
            if (cmd & 1) g_sfhss.cntOdd++; else g_sfhss.cntEven++;
            if (harvest_active) {
                /* trailer caught (parse already stored its data by parity):
                 * rejoin the main sequence 3 channels ahead of the anchor —
                 * 2 slots elapsed while lingering, the next is the 3rd. */
                next_channel(); next_channel(); next_channel();
                hop();
                harvest_active = 0;
            } else if ((cmd & 1) == 0 && ++harvest_count >= HARVEST_EVERY) {
                harvest_count = 0;
                harvest_active = 1;
                g_sfhss.cntHarvest++;
                harvest_t0 = g_sfhss.rtime;
                CC2500_Strobe(CC2500_SRX);  /* stay for the trailer */
            } else {
                next_channel();
                hop();
            }
        } else if (res == 2) {
            rx_restart();
        } else if (harvest_active) {
            if (elapsed(harvest_t0, now) > HARVEST_WINDOW_US) {
                /* no trailer this visit — rejoin the sequence. Push
                 * lastFrameTime 2 slots forward so the regular miss logic
                 * expects the NEXT frame at anchor+3 slots, not a stale one. */
                g_sfhss.lastFrameTime =
                    (g_sfhss.lastFrameTime + 2u * g_sfhss.interval) % TIME_WRAP;
                next_channel(); next_channel(); next_channel();
                hop_flush();
                harvest_active = 0;
            }
        } else {
            uint32_t e = elapsed(g_sfhss.lastFrameTime, now);
            uint32_t limit =
                g_sfhss.interval * (uint32_t)(g_sfhss.skipCount + 1) + HOPPING_TIMEOUT;
            if (e > limit) miss_frame();
        }
        break;
#endif
        /* FLYWHEEL (2026-07-12). The old discipline hopped only in reaction to
         * frames; on a missed DATA1 the fallback hopped at interval+500us past
         * the LAST frame — landing on every subsequent channel 500us AFTER its
         * DATA1's sync word, so a single miss decayed into a chronic chase, 30
         * blind hops (~205ms), a rebind camp on channel 0 (up to ~204ms), and a
         * 200-400ms servo freeze. (Same failure the retired mode 1 had.)
         * Now the receiver keeps its own slot clock: slotStart is the expected
         * prompt-read time of the current slot's DATA1, re-anchored by every
         * received frame and advanced by exactly `interval` on EVERY hop —
         * received or not. Blind hops arrive ON TIME (before the sync word),
         * so one miss costs one slot, not a desync. */
        if (res == 1) {
            if (!g_sfhss.frameThisSlot) g_sfhss.statRcv++;  /* once per slot */
            g_sfhss.frameThisSlot = 1;
            g_sfhss.skipCount = 0;
            /* Re-anchor the grid on measured timing (cancels crystal drift).
             * A drain containing DATA2 happened at/after DATA2-complete —
             * SLOT_DATA2_OFFSET_US later than a prompt DATA1 read; correct for
             * that so a late read doesn't drag the grid late. */
            if (got_data1 && !got_data2)
                g_sfhss.slotStart = g_sfhss.rtime;
            else
                g_sfhss.slotStart =
                    (g_sfhss.rtime + TIME_WRAP - SLOT_DATA2_OFFSET_US) % TIME_WRAP;
        } else if (res == 2) {
            rx_restart();               /* bad read: keep listening here */
        }
        /* Hop when the window past the (possibly future) anchor expires. */
        if (since_signed(g_sfhss.slotStart, now) > (int32_t)SLOT_2ND_WINDOW_US) {
            if (!g_sfhss.frameThisSlot) {
                g_sfhss.statLost++;
                if (++g_sfhss.skipCount >= FALLBACK_COUNT) {
                    g_sfhss.statResync++;
                    g_sfhss.phase = SFHSS_PH_BINDED; /* keep txaddr, re-acquire */
                    break;
                }
            }
            g_sfhss.frameThisSlot = 0;
            next_channel();
            hop_flush();
            g_sfhss.slotStart = (g_sfhss.slotStart + g_sfhss.interval) % TIME_WRAP;
        }
        break;
    }
}
