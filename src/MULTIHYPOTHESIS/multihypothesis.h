#ifndef MULTIHYPOTHESIS_H
#define MULTIHYPOTHESIS_H

#include "rtklib.h"

#define MAX_RTK_HYPOTHESES 5         /* max number of hypotheses */
#define MAX_RTK_QUEUE      150       /* max number of epochs stored */

#define SQR(x) ((x) * (x))

/* ------------------------------------------------------------------------- */
/* basic types ------------------------------------------------------------- */

typedef struct {

    int    length;                 /* in [0, MAX_RTK_QUEUE] */
    int    offset[MAX_RTK_QUEUE];  /* i-th element in the queue is stored in rtk[offset[i]] */
    rtk_t *rtk[MAX_RTK_QUEUE];     /* rtk structures (queue nodes) */

} rtk_queue_t;

typedef struct {

    int    target_solution_status; /* SOLQ_FIX, SOLQ_FLOAT, SOLQ_... */
    double solution_quality;       /* 0.0 is the best, negative value means 'is not defined' */
    rtk_queue_t *history;          /* rtk state for several consequtive epochs */
    
} rtk_history_t;

typedef struct {
    
    int      n_hypotheses;         /* number of active hypotheses */
    rtk_history_t *hypotheses[MAX_RTK_HYPOTHESES];
    rtk_t   *rtk_out;              /* resulting rtk solution structure to output */
    prcopt_t opt;                  /* processing options */
    
} rtk_multi_t;

typedef struct {

    obsd_t *obsd;                   /* observation data */
    int     n_obsd;                 /* number of data records */
    nav_t  *nav;                    /* navigation data */

} rtk_input_data_t;

typedef void (*rtk_multi_split_fpt)   (rtk_multi_t *, const rtk_input_data_t *);
typedef void (*rtk_multi_qualify_fpt) (rtk_multi_t *);
typedef void (*rtk_multi_merge_fpt)   (rtk_multi_t *);

typedef struct {
    
    rtk_multi_split_fpt   split;
    rtk_multi_qualify_fpt qualify;
    rtk_multi_merge_fpt   merge;
    
} rtk_multi_strategy_t;

/* -------------------------------------------------------------------------- */
/* manipulation functions --------------------------------------------------- */

/* rtk_history */

extern rtk_history_t *rtk_history_init(prcopt_t opt);
extern int rtk_history_is_valid(const rtk_history_t *rtk_history);
extern void rtk_history_free(rtk_history_t *rtk_history);

extern void rtk_history_add(rtk_history_t *rtk_history, const rtk_t *rtk);
extern void rtk_history_cut(rtk_history_t *rtk_history, int n_cut);
extern int  rtk_history_is_empty(const rtk_history_t *rtk_history);
extern void rtk_history_clear(rtk_history_t *rtk_history);
extern rtk_t *rtk_history_get_pointer(const rtk_history_t *rtk_history, int index);
extern rtk_t *rtk_history_get_pointer_to_last(const rtk_history_t *rtk_history);

/* rtk_input_data */

extern int rtk_input_data_is_valid(const rtk_input_data_t *rtk_input_data);

/* rtk_multi */

extern rtk_multi_t *rtk_multi_init(prcopt_t opt);
extern int rtk_multi_is_valid(const rtk_multi_t *rtk_multi);
extern void rtk_multi_free(rtk_multi_t *rtk_multi);

extern int rtk_multi_add(rtk_multi_t *rtk_multi, rtk_t *rtk);
extern void rtk_multi_exclude(rtk_multi_t *rtk_multi, int index);

extern void rtk_multi_estimate(rtk_multi_t *rtk_multi,
                               const rtk_multi_strategy_t *rtk_multi_strategy, 
                               const rtk_input_data_t *rtk_input_data);

/* rtk_multi_strategy */

extern int rtk_multi_strategy_is_valid(const rtk_multi_strategy_t *rtk_multi_strategy);

#endif /* #ifndef MULTIHYPOTHESIS_H */
