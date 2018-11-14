import serial
import scipy.optimize
import numpy as np

"""
Arm Command class
Represents a single command to be executed on the FANUC
Serializable

"""
class ArmCommand():

	TYPE_STATE_ON = "on"
	TYPE_STATE_OFF = "off"
	TYPE_STATE_LISTEN = "listen"
	TYPE_MOVE_JOINT = "move"

	def __init__(self,type,component=0,value=0):

		self.type = type
		self.component = component
		self.value = value


	def serialize(self):

		val_type = 1 if self.type==self.TYPE_MOVE_JOINT else 0

		byte_type = chr(val_type)
		byte_comp = chr(self.component)
		byte_val  = chr(self.value)

		return byte_type + byte_comp + byte_val
		
"""
Controller class
Communicates with the FANUC over serial connection
Sends small serialized commands to update state information on FANUC controller
"""
class ArmController():
	
	def __init__(self):
		self.serial = serial.Serial(
			port='/dev/ttyUSB1',
			baudrate=9600,
			parity=serial.PARITY_ODD,
			stopbits=serial.STOPBITS_TWO,
			bytesize=serial.SEVENBITS
		)
		self.serial.timeout = 0.5
		

	def connect(self):
		self.serial.open()


	def sendCommand(self,cmd):
		print(cmd.serialize(),bytes(cmd.serialize()))
		# self.serial.write(cmd.serialize())




def inverse_kinematic_optimization(chain, target_frame, starting_nodes_angles, regularization_parameter=None, max_iter=None):
    """
    Computes the inverse kinematic on the specified target with an optimization method
    Parameters
    ----------
    chain: ikpy.chain.Chain
        The chain used for the Inverse kinematics.
    target: numpy.array
        The desired target.
    starting_nodes_angles: numpy.array
        The initial pose of your chain.
    regularization_parameter: float
        The coefficient of the regularization.
    max_iter: int
        Maximum number of iterations for the optimisation algorithm.
    """
    # Only get the position
    target = target_frame[:3, 3]

    if starting_nodes_angles is None:
        raise ValueError("starting_nodes_angles must be specified")

    # Compute squared distance to target
    def optimize_target(x):
        # y = np.append(starting_nodes_angles[:chain.first_active_joint], x)
        y = chain.active_to_full(x, starting_nodes_angles)
        squared_distance = np.linalg.norm(chain.forward_kinematics(y)[:3, -1] - target)
        return squared_distance

    # If a regularization is selected
    if regularization_parameter is not None:
        def optimize_total(x):
            regularization = np.linalg.norm(x - starting_nodes_angles[chain.first_active_joint:])
            return optimize_target(x) + regularization_parameter * regularization
    else:
        def optimize_total(x):
            return optimize_target(x)

    # Compute bounds
    real_bounds = [link.bounds for link in chain.links]
    # real_bounds = real_bounds[chain.first_active_joint:]
    real_bounds = chain.active_from_full(real_bounds)

    options = {}
    # Manage iterations maximum
    if max_iter is not None:
        options["maxiter"] = max_iter

    # Utilisation d'une optimisation L-BFGS-B
    res = scipy.optimize.minimize(optimize_total, chain.active_from_full(starting_nodes_angles), method='L-BFGS-B', bounds=real_bounds, options=options)

    return(chain.active_to_full(res.x, starting_nodes_angles))
    # return(np.append(starting_nodes_angles[:chain.first_active_joint], res.x))



controller = ArmController()

controller.connect()
controller.sendCommand(ArmCommand(ArmCommand.TYPE_STATE_ON))
controller.sendCommand(ArmCommand(ArmCommand.TYPE_STATE_LISTEN))

angles = np.array()
def moveArm(controller,pos):

	angles = inverse_kinematic_optimization(ikpy.chain.Chain(),pos,angles)
	for i,a in enumerate(angles):
		controller.sendCommand(ArmCommand(ArmCommand.TYPE_MOVE_JOINT,i,a))






moveArm((1,0,0));
