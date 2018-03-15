/* 
 *   FIX-AND-HOLD refinement strategy (FXHR)
 */

#include "fix_and_hold_refinement_strategy.h"

double ratio_float = 0.0;

extern rtk_multi_t *rtk_multi_init_fxhr(prcopt_t opt)
{
    rtk_multi_t   *rtk_multi = rtk_multi_init(opt);
    rtk_t         *rtk;
    int index_new;

    /* init float filter */
    opt.modear = ARMODE_OFF;
    opt.gpsmodear = ARMODE_OFF;
    opt.glomodear = GLO_ARMODE_OFF;
    opt.bdsmodear = ARMODE_OFF;
    rtk = rtk_init(&opt);
    index_new = rtk_multi_add(rtk_multi, rtk);
    rtk_multi->hypotheses[index_new]->target_solution_status = SOLQ_FLOAT;
    
    assert( index_new == 0 );
    
    rtk_free(rtk);

    return rtk_multi;
}

extern int rtk_multi_is_valid_fxhr(const rtk_multi_t *rtk_multi)
{
    int i;
    
    if ( !rtk_multi_is_valid(rtk_multi) ) {

        return 0;
    }
    if ( rtk_multi->n_hypotheses > N_HYPOTHESES_FXHR ) {
        
        return 0;
    }
    /* float filter (hypothesis) should be defined */
    if ( rtk_history_is_empty(rtk_multi->hypotheses[0]) )  {

        return 0;
    }
    if ( rtk_multi->hypotheses[0]->target_solution_status != SOLQ_FLOAT ) {

        return 0;
    }
    for (i = 1; i < N_HYPOTHESES_FXHR; i++) {

        if ( rtk_history_is_empty(rtk_multi->hypotheses[i]) ) continue;
        if ( rtk_multi->hypotheses[i]->target_solution_status != SOLQ_FIX ) {

            return 0;
        }
    }

    return 1;
}

static int are_solutions_close(const sol_t *sol_1, const sol_t *sol_2)
{
    double pos_1[3];
    double pos_2[3];
    double delta;

    assert(sol_1 != NULL);
    assert(sol_2 != NULL);

    memcpy(pos_1, sol_1->rr, sizeof(double) * 3);
    memcpy(pos_2, sol_2->rr, sizeof(double) * 3);

    delta = sqrt(SQR(pos_2[0]-pos_1[0]) + SQR(pos_2[1]-pos_1[1]) + SQR(pos_2[2]-pos_1[2]));
    if ( delta < RTK_POS_THRESH_FXHR ) {

        return 1;
    };

    return 0;
}

static double get_fix_fraction(rtk_history_t *rtk_history)
{
    int i, n_epochs;
    int n_fix_epochs = 0;
    double fix_fraction;
    rtk_t *rtk;

    n_epochs = rtk_history->history->length;
    
    if ( n_epochs < 2 ) return 1.0; 

    /* last epoch is allowed to be float and is not accounted */
    for (i = 0; i < (n_epochs-1); i++) {
            
        rtk = rtk_history_get_pointer(rtk_history, i);
        if ( rtk->sol.stat != SOLQ_FIX ) continue;
        n_fix_epochs++;
    }
    fix_fraction = ((double) n_fix_epochs) / (n_epochs - 1);

    return fix_fraction;
}

static int get_number_of_alternative_fixes(rtk_history_t *rtk_history)
{
    int i, n_epochs;
    int n_alternative_fixes = 0;
    rtk_t *rtk;

    n_epochs = rtk_history->history->length;

    for (i = 0; i < n_epochs; i++) {
        
        rtk = rtk_history_get_pointer(rtk_history, i);
        if ( rtk->sol.stat != SOLQ_FIX ) {
            
            if ( rtk->is_alternative_fix_possible ) n_alternative_fixes++;
            continue;
        }

        if ( rtk->is_alternative_fix_possible ) {
            
            if ( are_solutions_close(&rtk->sol, &rtk->sol_alternative) ) {
                
               rtk->is_alternative_fix_possible = 0;
            }
            else {
                
                n_alternative_fixes++;
            }
        }
    }

    return n_alternative_fixes;
}

static double get_rms_residuals_fix(rtk_history_t *rtk_history)
{
    int i, sat, freq, n_epochs;
    int n_fix_obs = 0;
    double sqr_residuals_fix = 0.0;
    double rms_residuals_fix = 0.0;
    rtk_t *rtk;

    n_epochs = rtk_history->history->length;
    
    for (i = 0; i < n_epochs; i++) {
        
        rtk = rtk_history_get_pointer(rtk_history, i);
        if ( rtk->sol.stat != SOLQ_FIX ) continue;
        
        for (sat = 0; sat < MAXSAT; sat++) {
            for (freq = 0; freq < rtk->opt.nf; freq++) {
                
                if ( rtk->ssat[sat].vsat[freq] == 0 ) continue;   /* satellite is not valid */
                if ( (rtk->ssat[sat].fix[freq] != 2)
                  && (rtk->ssat[sat].fix[freq] != 3) ) continue;  /* satellite is not used for fix */
                if ( rtk->ssat[sat].resc[freq] == 0.0 
                  && rtk->ssat[sat].resp[freq] == 0.0 ) continue; /* residuals are not defined */
                
                sqr_residuals_fix += SQR(rtk->ssat[sat].resc[freq]);
                n_fix_obs++;
            }
        }
    }
    
    if ( n_fix_obs > 0 ) rms_residuals_fix = sqrt(sqr_residuals_fix / n_fix_obs);

    return rms_residuals_fix;
}

/* return: -1 : validation impossible, 0 : not valid, 1 : valid */
static int rtk_history_validate_fxhr(rtk_history_t *rtk_history)
{
    int n_epochs;
    int n_alternative_fixes;
    double rms_residuals_fix;
    double fix_fraction;

    assert( rtk_history_is_valid(rtk_history) );
    assert( !rtk_history_is_empty(rtk_history) );
    
    n_epochs = rtk_history->history->length;
    
    rtk_history->solution_quality = -1.0;

    if ( rtk_history->target_solution_status != SOLQ_FIX ) { /* validate only 'fix' hypotheses */
        
        return -1;
    }
    if ( n_epochs >= 2 ) {

        fix_fraction = get_fix_fraction(rtk_history);
        if ( fix_fraction < MIN_FIX_FRACTION_FXHR ) {

            rtk_history->solution_quality = LOW_SOL_QUAL + 1.0;
            return -1;
        }
    }
    if ( n_epochs < MIN_EPOCHS_FXHR ) {

        return -1;
    }

    n_alternative_fixes = get_number_of_alternative_fixes(rtk_history);
    rms_residuals_fix   = get_rms_residuals_fix(rtk_history);

    rtk_history->solution_quality = rms_residuals_fix;
    
    return (rms_residuals_fix < RESID_THRESH_FXHR) 
        && ((n_alternative_fixes < MIN_ALTERNATIVE_FIXES_FXHR) 
            || (rms_residuals_fix < RESID_FINE_TRESH_FXHR));
}

extern void rtk_multi_split_fxhr(rtk_multi_t *rtk_multi, const rtk_input_data_t *rtk_input_data)
{
    static int split_outage = -1;
    rtk_history_t *hypothesis;
    int index_new;
    int is_fix_possible = 0; 
    int is_fix_hypothesis_of_low_quality = 0;
    rtk_t *rtk;
    rtk_t *rtk_tmp;
    
    assert( rtk_multi_is_valid_fxhr(rtk_multi) );
    assert( rtk_input_data_is_valid(rtk_input_data) );
    
    split_outage++;
    if ( split_outage < SPLIT_INTERVAL_FXHR ) {

        return;
    }

    split_outage = 0;

    /* check if fix is possible */
    rtk = rtk_history_get_pointer_to_last(rtk_multi->hypotheses[0]);
    rtk_tmp = rtk_init(&rtk_multi->opt);
    rtk_copy(rtk, rtk_tmp);
    rtk_tmp->opt = rtk_multi->opt;
    rtkpos(rtk_tmp, rtk_input_data->obsd, rtk_input_data->n_obsd, rtk_input_data->nav);
    is_fix_possible = (rtk_tmp->sol.stat == SOLQ_FIX);
    ratio_float = rtk_tmp->sol.ratio;
    
    /* check is there a fix hypothesis */
    if ( rtk_multi->n_hypotheses >= N_HYPOTHESES_FXHR ) {
        
        hypothesis = rtk_multi->hypotheses[N_HYPOTHESES_FXHR-1];
        rtk = rtk_history_get_pointer_to_last(hypothesis);
        
        /* check if 'fix' hypothesis is weak or not */
        is_fix_hypothesis_of_low_quality = (rtk->sol.stat != SOLQ_FIX) 
                                        && (hypothesis->solution_quality >= LOW_SOL_QUAL);
        
        /* add alternative fix record */
        if ( is_fix_possible ) {
            
            rtk->is_alternative_fix_possible = 1;
            rtk->sol_alternative = rtk_tmp->sol;
        }
        else {
            
            rtk->is_alternative_fix_possible = 0;
        }
        
        if ( !is_fix_hypothesis_of_low_quality ) {
            
            rtk_free(rtk_tmp);
            return;
        }
    }

    if ( !is_fix_possible ) {

        rtk_free(rtk_tmp);
        return;
    }

    /* initialize new hypothesis structure */
    rtk = rtk_history_get_pointer_to_last(rtk_multi->hypotheses[0]);
    rtk_copy(rtk, rtk_tmp);
    rtk_tmp->opt = rtk_multi->opt;
    
    /* add new hypothesis if all checks are passed */
    if ( is_fix_hypothesis_of_low_quality ) {

        rtk_multi_exclude(rtk_multi, N_HYPOTHESES_FXHR-1); /* kill weak hypothesis */
    }
    index_new = rtk_multi_add(rtk_multi, rtk_tmp);
    rtk_multi->hypotheses[index_new]->target_solution_status = SOLQ_FIX;

    rtk_free(rtk_tmp);
}

extern void rtk_multi_qualify_fxhr(rtk_multi_t *rtk_multi)
{
    int i;
    rtk_history_t *hypothesis;

    assert( rtk_multi_is_valid_fxhr(rtk_multi) );
    
    /* exclude bad hypotheses */
    for (i = 1; i < N_HYPOTHESES_FXHR; i++) { /* note: i == 0 is for float filter (never excluded) */
        
        hypothesis = rtk_multi->hypotheses[i];
        if ( rtk_history_is_empty(hypothesis) ) continue;

        if ( rtk_history_validate_fxhr(hypothesis) == 0 ) {
            
            rtk_multi_exclude(rtk_multi, i);
        }
    }
    
}

extern void rtk_multi_merge_fxhr(rtk_multi_t *rtk_multi)
{
    rtk_history_t *hypothesis = NULL;
    rtk_t *rtk = NULL;

    assert( rtk_multi_is_valid_fxhr(rtk_multi) );

    if ( rtk_multi->n_hypotheses == 1 ) { /* output float */

        hypothesis = rtk_multi->hypotheses[0];
        rtk = rtk_history_get_pointer_to_last(hypothesis);
        rtk->sol.ratio = ratio_float;
    }
    
    if ( rtk_multi->n_hypotheses > 1 ) { /* output fix */
        
        hypothesis = rtk_multi->hypotheses[1];
        rtk = rtk_history_get_pointer_to_last(hypothesis);
    }
    
    assert( rtk_history_is_valid(hypothesis) );
    
    rtk_copy(rtk, rtk_multi->rtk_out);
}
