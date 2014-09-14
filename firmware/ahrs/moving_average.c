#include <stdint.h>
#include "arm_math.h"
#include <string.h>

#include "vector_space.h"
#include "moving_average.h"

void vector3d_simple_moving_average(vector3d_f_t new_sampling_data, vector3d_f_t *data_fifo,
	vector3d_f_t *data_result, int sampling_count)
{
	int i;
	for(i = 0; i < (sampling_count - 1); i++) {
		//Drop the old data
		memcpy((uint8_t *)&data_fifo[i], (uint8_t *)&data_fifo[i + 1], sizeof(vector3d_f_t));

		//Calculate sum of old data without last time dropped
		data_result->x += data_fifo[i].x;
		data_result->y += data_fifo[i].y;
		data_result->z += data_fifo[i].z;
	}

	/* Put new data at the last of the FIFO */
	memcpy((uint8_t *)&data_fifo[sampling_count -1], (uint8_t *)&new_sampling_data, sizeof(vector3d_f_t));

	//Add the new data into the FIFO sum
	data_result->x += new_sampling_data.x;
	data_result->y += new_sampling_data.y;
	data_result->z += new_sampling_data.z;

	/* Caluculate the average of the new FIFO */
	data_result->x /= sampling_count;
	data_result->y /= sampling_count;
	data_result->z /= sampling_count;
}

void vector3d_weight_moving_average(vector3d_f_t new_sampling_data, vector3d_f_t *data_fifo,
	vector3d_f_t *data_result, int sampling_count)
{
	int i;
	for(i = 0; i < (sampling_count - 1); i++) {
		//Drop the old data
		memcpy((uint8_t *)&data_fifo[i], (uint8_t *)&data_fifo[i + 1], sizeof(vector3d_f_t));

		//weight the old data without last time dropped then calculate the sum
		data_result->x += data_fifo[i].x * (i + 1);
		data_result->y += data_fifo[i].y * (i + 1);
		data_result->z += data_fifo[i].z * (i + 1);
	}

	/* Put new data at the last of the FIFO */
	memcpy((uint8_t *)&data_fifo[sampling_count -1], (uint8_t *)&new_sampling_data, sizeof(vector3d_f_t));

	//Weight the new data then add into the FIFO sum
	data_result->x += (new_sampling_data.x * sampling_count);
	data_result->y += (new_sampling_data.y * sampling_count);
	data_result->z += (new_sampling_data.z * sampling_count);

	/* Caluculate the average of the new FIFO */
	data_result->x /= (sampling_count * (sampling_count + 1) / 2);
	data_result->y /= (sampling_count * (sampling_count + 1) / 2);
	data_result->z /= (sampling_count * (sampling_count + 1) / 2);
}

void vector3d_exponential_moving_average(vector3d_f_t new_sampling_data, vector3d_f_t *data_fifo,
	vector3d_f_t *data_result, int sampling_count)
{
	float weight;
	float alpha, alpha_sum = 0;

	int i;
	for(i = 0; i < (sampling_count - 1); i++) {
		//Drop the old data
		memcpy((uint8_t *)&data_fifo[i], (uint8_t *)&data_fifo[i + 1], sizeof(vector3d_f_t));

		//Calculate the alpha value
		alpha = 2 / (i + 2);

		//Caluclate the weight value
		weight = pow(1 - alpha, i + 1);

		//Calculate the alpha sum
		alpha_sum += weight;

		//weight the old data without last time dropped then calculate the sum
		data_result->x += data_fifo[i].x * weight;
		data_result->y += data_fifo[i].y * weight;
		data_result->z += data_fifo[i].z * weight;
	}

	/* Put new data at the last of the FIFO */
	memcpy((uint8_t *)&data_fifo[sampling_count -1], (uint8_t *)&new_sampling_data, sizeof(vector3d_f_t));

	//Calculate the alpha value
	alpha = 2 / (sampling_count + 1);

	//Caluclate the weight value
	weight = pow(1 - alpha, i + 1);

	//Calculate the alpha sum
	alpha_sum += weight;

	//Add the new data into the FIFO sum
	data_result->x += new_sampling_data.x * weight;
	data_result->y += new_sampling_data.y * weight;
	data_result->z += new_sampling_data.z * weight;

	/* Caluculate the average of the new FIFO */
	data_result->x /= alpha_sum;
	data_result->y /= alpha_sum;
	data_result->z /= alpha_sum;
}
