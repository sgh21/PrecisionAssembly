import numpy as np
def spin_all(theta,step=4):
    theta_list = []
    for i in range(step):
        theta_list.append(theta+i*2*np.pi/step)
    for i,theta_obj in enumerate(theta_list):
        theta_list[i] = theta_obj-2*np.pi if theta_obj > np.pi else theta_obj
    return theta_list

def get_close(theta_refer,theta_list):
    theta_list = np.array(theta_list)
    theta_delta_list = np.abs(theta_list - theta_refer)
    return np.argmin(theta_delta_list)
if __name__ == "__main__":
    a = -np.pi/4
    b = -np.pi*3/4
    delta_theta = ((a-b)+2*np.pi)% (2*np.pi)
    flag = (delta_theta<=np.pi)
    print(flag,delta_theta)
    theta = -np.pi/3
    theta_refer = np.pi/4
    print(spin_all(theta,4))
    print(get_close(theta_refer,spin_all(theta,4)))