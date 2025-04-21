from sim import *


if __name__ == "__main__":
    # Create an instance of the RobotSkySim class
    sim = SimBase()

    # add action subscriber
    # add state publisher

    sim.initialize()

    # Run the simulation
    while sim.is_running():
        state = sim.get_state()
        sim.set_action([])

        # send state msg

        sim.step()
        sim.render()
