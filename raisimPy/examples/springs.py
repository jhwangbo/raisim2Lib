import os
import raisimpy as raisim
import time


raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
world = raisim.World()
world.setTimeStep(0.001)

cartpole_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/springDamper/cartpole.urdf"
chain_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/springDamper/chainSpringed.urdf"

server = raisim.RaisimServer(world)
world.addGround()

rev_and_pris_spring_and_damper = world.addArticulatedSystem(cartpole_urdf_file)
ball_spring_and_damper = world.addArticulatedSystem(chain_urdf_file)

rev_and_pris_spring_and_damper.setName("rev_pris_joint")
ball_spring_and_damper.setName("ball_joint")

server.launchServer(8080)

for i in range(500000):
    time.sleep(0.001)
    server.integrateWorldThreadSafe()

server.killServer()
