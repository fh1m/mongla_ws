# Research owed — the questions the sweeps could not answer

> Written so a future session **re-runs exactly what is missing** instead of re-deriving what is
> already known. Each row names the question, why it went unanswered, and what would settle it.
> States: `[ ]` open · `[~]` partially answered · `[x]` answered (finding landed in a dossier).

**Why so many rows are open:** the 2026-09-22 sweeps exhausted the session-wide web-search budget
(200/200) partway through. The last three ran with **zero search**, working only through the arXiv
API, Semantic Scholar and direct fetches; search engines via fetch returned CAPTCHAs. Everything
below is **very likely answerable** with a working quota — these are not dead ends.

---

## Highest value first

| # | question | why it is open | what settles it |
|---|---|---|---|
| R-1 | **Allan variance for a BNO085-class IMU** | datasheet URLs 404'd; no search to find alternatives | ⚠ **Do not research this — measure it.** [`BENCH.md` B-6](BENCH.md) is a 12-hour run on our own part, which beats any datasheet |
| R-2 | **The T200's `k` and its advance-ratio slope** | [Lam et al., OCEANS 2023](https://ieeexplore.ieee.org/document/10244513/) is the right paper, paywalled. Only the qualitative statement (K_T falls linearly with J, zero at geometric pitch) was reachable | the paper, or our own load-cell run ([B-3](BENCH.md)) plus a towed measurement we cannot yet make |
| R-3 | **RoboSub / SAUVC technical design reports** | grey literature; not indexed by arXiv or Semantic Scholar, and search was unavailable. The perception sweep *did* reach five TDRs earlier, while search still worked | a working search quota, or direct archive URLs |
| R-4 | **Underwater kidnapped-robot / relocalization** | the arXiv query returned an unrelated paper | a proper search; this is the only route to *"how does an AUV recover an absolute fix after a blackout"* |
| R-5 | **A Pi-5-class CPU cost for fixed-lag smoothing** | **no paper publishes one** — T-ESKF benchmarked on a desktop R9 7950X, the factor-graph work reports no timing at all | ⚠ may be genuinely unpublished. Our own measurement is the answer |
| R-6 | **A Hailo-8 number for UniDepth V2** | it is the only metric-depth model that survives water (AbsRel 0.093–0.116) and no Hailo-8 figure exists | the Hailo model zoo, a vendor answer, or compiling it ourselves |
| R-7 | **Classical marine system ID** — Fossen coefficient practice, Ridao/Girona, Hegrenæs & Hallingstad (HUGIN model-aided INS), Martin & Whitcomb, panel-code validation, coast-down Cd | this literature lives in IEEE/Elsevier/MDPI, which arXiv cannot substitute for | a working search plus access, or the textbooks |
| R-8 | **The single-beacon error figure** (arXiv 2110.13825, CSAC + USBL, validated against LBL on 3 AUVs) | the number is in the 34-page body, not the abstract | fetch the PDF body |
| R-9 | **Masks vs boxes for alignment error** | ⚠ **appears genuinely unstudied.** The field routes around it: oriented boxes (Cornell), keypoints (CMU), features + PnP (Bumblebee) | our own A/B on recorded footage would be a first |
| R-10 | **Human-minutes per class for model production** | **an empty cell across the entire 2026 auto-labelling literature** | [`BENCH.md` B-9](BENCH.md) — if we time ours, the number does not exist anywhere else |
| R-11 | **SAM 3 cost per image** | a **97× discrepancy** between two vendor figures on the same H200 could not be reconciled | a careful reading of both sources, or our own benchmark |
| R-12 | **XFeat on a Raspberry Pi 5** | the published figure is an i5-1135G7 at VGA | measure it on our Pi |

---

## The pattern worth noticing

**Five of these (R-1, R-5, R-9, R-10, R-12) are not literature gaps — they are measurements
nobody has published.** For a team with a bench, that is an opportunity rather than an obstacle:
each one we measure is a number that does not currently exist in the field.

## How to re-run a sweep

Sonnet research agents, **≤3 in parallel**, each told to write **incrementally** to
`.claude/context/sota/sources/<topic>.md` — create the file after the first 3 sources and append
every 4–6 after. Two sessions were lost to agents holding findings in their heads and then hitting
a limit.
