#ifndef __FILTER_SMA_H
#define __FILTER_SMA_H

void vector3d_simple_moving_average(vector3d_f_t new_sampling_data, vector3d_f_t *data_fifo,
	vector3d_f_t *data_result, int sampling_count);

#endif
