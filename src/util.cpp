#include "util.hpp"

void calc_delayed_est(DelayedEstState &state, kfloat_t reading)
{
	if (state.count > DELAYED_EST_SAMPLES) {
		state.old_old_est = state.old_est;
		state.old_est = state.est;
		state.count = 1;
	}
	// Compute a running average
	state.est *= (float)state.count / (state.count + 1);
	state.est += reading / (state.count + 1);
	state.count += 1;
}
