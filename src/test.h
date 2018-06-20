#pragma once
#include "rtklib.h"
int restore_missed_sats(rtk_t *rtk, obsd_t *obs, int n_rover, const nav_t *nav, int n_all);
void post_update_obs(rtk_t *rtk, const nav_t *nav, obsd_t *obs_cur, int n_obs_cur, double *cur_pos);
