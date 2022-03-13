from PathPlanning.RRT_Star import *
from zmqRemoteApi.clients.python.zmqRemoteApi import RemoteAPIClient

def main():
    client = RemoteAPIClient()
    simulation = client.getObject('sim')
    #clientId = sim.startSimulation()
    planRRTStarPath(simulation)


if __name__ == "__main__":
    main()