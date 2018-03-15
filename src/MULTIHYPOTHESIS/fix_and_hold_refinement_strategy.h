/* 
 *   FIX-AND-HOLD refinement strategy (FXHR)
 */

#ifndef FIX_AND_HOLD_REFINEMENT_STRATEGY_H
#define FIX_AND_HOLD_REFINEMENT_STRATEGY_H

#include "rtklib.h"
#include "multihypothesis.h"

#define N_HYPOTHESES_FXHR     2      /* 'float' and 'fix' hypotheses */
#define MIN_EPOCHS_FXHR       100    /* min number of epochs for hypothesis to validate */
#define MIN_FIX_FRACTION_FXHR 0.70   /* min fix fraction for hypothesis to validate */
#define RESID_THRESH_FXHR     0.02   /* max RMSE of phase residuals allowed for 'fix hypothesis' 
                                        to been mark as valid [m] */
#define SPLIT_INTERVAL_FXHR   10     /* time span between split operations (epochs) */
#define LOW_SOL_QUAL          100.0  /* low solution quality threshold (rtk_history->solution_quality) */

#define RTK_POS_THRESH_FXHR        0.1    /* min 3D distance between solutions to distinguish */
#define MIN_ALTERNATIVE_FIXES_FXHR 10     /* min number of alternative fixes (epochs) 
                                             to discard fix hypothesis */
#define RESID_FINE_TRESH_FXHR      0.01   /* don't discard fix if RMS of phase residuals is fine [m] */

extern rtk_multi_t *rtk_multi_init_fxhr(prcopt_t opt);
extern int rtk_multi_is_valid_fxhr(const rtk_multi_t *rtk_multi);

extern void rtk_multi_split_fxhr(rtk_multi_t *rtk_multi, const rtk_input_data_t *rtk_input_data);
extern void rtk_multi_qualify_fxhr(rtk_multi_t *rtk_multi);
extern void rtk_multi_merge_fxhr(rtk_multi_t *rtk_multi);

#endif /* #ifndef FIX_AND_HOLD_REFINEMENT_STRATEGY_H */
