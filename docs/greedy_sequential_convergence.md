# Greedy Sequential — Convergence Design Notes

## Convergence rule

A drone declares convergence when **both** conditions are true simultaneously:

### Condition 1 — Work is done (either of)

- **Bundle full:** `my_claims_.size() >= bundle_size_`
- **All items accounted for:** `claimed_by_others_.size() + my_claim_names_.size() >= auction_items_.size()`

This is a **per-drone** criterion. Each drone converges independently based on its own local view. A global target (e.g. `n_drones × bundle_size`) was considered and rejected — see the rationale below.

### Condition 2 — Network is stable

No bid has been **received** for at least `stability_wait_s_ = 0.5s`.

In plain terms: **"I have all the tasks I'm going to get, AND the network has gone quiet."**

---

## How condition 2 is implemented

`last_activity_time_` is a `steady_clock` timestamp that is advanced **only when a bid is received** (inside `update()`). The stability idle time is:

```
idle = now - last_activity_time_
```

`check_convergence()` returns `false` until `idle >= stability_wait_s_`.

Critically, `last_activity_time_` is **not** reset when this drone sends or retransmits a bid (`on_run()`, conflict retransmit in `update()`). Those paths only update `last_bid_time_`, which is the retransmit interval timer — a separate clock.

### Why only incoming bids reset the clock

If own retransmits reset `last_activity_time_`, a drone that keeps retransmitting with no peer response would push the clock forward on every retransmit cycle and the 0.5s window would never expire. This happens in practice when all peers have already converged and called `reset()` — they stop responding entirely. The result is a drone stuck retransmitting forever.

By advancing the clock only on **incoming** bids, the stability window measures *"how long since anyone last talked to me"*, which correctly expires even when the drone is still retransmitting into silence.

---

## Why a per-drone criterion instead of a global one

An earlier version required a global target: `total_claimed >= n_drones × bundle_size`. This caused a deadlock:

1. Fast drones converge, call `reset()`, and stop sending bids.
2. A late drone has an incomplete `claimed_by_others_` set (missed some messages).
3. The late drone can never reach the global target because the peers it needs to hear from have gone silent.
4. The late drone retransmits forever — the stuck-drone bug.

Per-drone convergence breaks this cycle: each drone only needs to account for items it has personally observed, and the stability wait absorbs any remaining in-flight messages before it finalises.

---

## Conflict resolution and the stability wait interaction

When this drone wins a conflict, it immediately broadcasts all current claims (not just the contested one) so the loser can yield before `check_convergence()` fires. The loser receives that retransmit, updates `last_activity_time_`, yields the item, and sends a new bid. Both sides restart their stability clocks, giving the network time to fully settle before either drone converges.
