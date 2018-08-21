import matplotlib.pyplot as plt
import pylab
import numpy as N
import scipy.io as sio
import math
from pyfmi import load_fmu

fmu_loc = '/home/shashank/Documents/Gap Year Work/TAMU_ROVm/ROVm/Resources/FMU/'
fmu_sm_name = 'SimplifiedBlueROV2.fmu'
fmu_fm_name = 'InputBasedBlueROV2.fmu'
fmu_full_sm_name = fmu_loc + fmu_sm_name
fmu_full_fm_name = fmu_loc + fmu_fm_name
smmodel = load_fmu(fmu_full_sm_name)
fmmodel = load_fmu(fmu_full_fm_name)

# logInfo = {1:[1,4],2:[4,7],3:[5,4],4:[6,10],5:[15,17],6:[16,17]}
logInfo = {1:[16,1],}
for i in range(1,len(logInfo)+1):
	for j in range(1,logInfo[i][1]+1):
		logNumber = [logInfo[i][0], 10]
		exp_log_loc = '/home/shashank/Documents/Gap Year Work/TAMU_ROVm/ROVm/Resources/PARSED_DATA_SPLIT/LOG' + str(logNumber[0]) + '/'
		exp_log_handle = 'IN_OUT_LOG' + str(logNumber[0]) + '_PARSED_' + str(logNumber[1]) + '.mat'
		exp_log_name = exp_log_loc + exp_log_handle
		exp = sio.loadmat(exp_log_name)

		parsed_exp = exp['inout_cell_mat_parsed']
		t = parsed_exp[:,0]
		in_ch1 = parsed_exp[:,1]
		in_ch2 = parsed_exp[:,2]
		in_ch3 = parsed_exp[:,3]
		in_ch4 = parsed_exp[:,4]
		in_ch5 = parsed_exp[:,5]
		in_ch6 = parsed_exp[:,6]
		u_traj = N.transpose(N.vstack((t,in_ch1, in_ch2, in_ch3, in_ch4, in_ch5, in_ch6)))

		t_end_index = int(math.floor(parsed_exp.shape[0]-1))
		t_data = parsed_exp[0:t_end_index,0]
		t_end = math.floor(t_data[t_data.shape[0]-1])
		v_x_data = parsed_exp[0:t_end_index,7]
		v_y_data = parsed_exp[0:t_end_index,9]
		v_z_data = parsed_exp[0:t_end_index,8]

		try:
			input_object = (['u[1]','u[2]','u[3]','u[4]','u[5]','u[6]'], u_traj)
			smmodel.set('rovBody.mu_d',500)
			res_sm = smmodel.simulate(final_time = t_end, input=input_object)
			v_x_sm = res_sm['absoluteVelocity.v[1]']
			v_y_sm = res_sm['absoluteVelocity.v[2]']
			v_z_sm = res_sm['absoluteVelocity.v[3]']
			t_sm = res_sm['time']

			res_fm = fmmodel.simulate(final_time = t_end, input=input_object)
			v_x_fm = res_fm['absoluteVelocity.v[1]']
			v_y_fm = res_fm['absoluteVelocity.v[2]']
			v_z_fm = res_fm['absoluteVelocity.v[3]']
			t_fm = res_fm['time']

			plt.figure(1)
			plt.figure(figsize=(19.2,10.8), dpi=100)
			plt.subplot(3,1,1)
			plt.plot(t_sm, v_x_sm, t_fm, v_x_fm)
			plt.legend(('Simplified', 'Full'))
			plt.title("Model Comparison | Testing the Simplified and Full Model on Data Set: " + str(logNumber[0]) + "." + str(logNumber[1]))
			plt.ylabel("X-Axis (m/s)")
			plt.grid(True)

			plt.subplot(3,1,2)
			plt.plot(t_sm, v_y_sm, t_fm, v_y_fm)
			plt.legend(('Simplified', 'Full'))
			plt.ylabel("Y-Axis (m/s)")
			plt.grid(True)

			plt.subplot(3,1,3)
			plt.plot(t_sm, v_z_sm, t_fm, v_z_fm)
			plt.legend(('Simplified', 'Full'))
			plt.ylabel("Z-Axis (m/s)")
			plt.xlabel("Time (s)")
			plt.grid(True)

			pylab.savefig("Comp_"+str(logNumber[0]) + '_' + str(logNumber[1]) + '.png', bbox_inches = 'tight')
		except:
			print("Error in simulating Log " + str(logNumber[0]) + "." + str(logNumber[1]))