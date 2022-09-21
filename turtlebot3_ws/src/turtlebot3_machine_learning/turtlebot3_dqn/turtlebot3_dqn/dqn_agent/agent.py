
import algo_dqn.py ## algo_xxx.py

import rclpy
from rclpy.node import Node

from turtlebot3_msgs.srv import Dqn

class Agent(Node):
    def __init__(self, algo):
        super().__init__('agent')
        self.algo = algo ## algo='algo_dqn'
        
        """************************************************************
        ** Initialise ROS clients
        ************************************************************"""
        # Initialise clients
        self.dqn_com_client = self.create_client(Dqn, 'dqn_com')

        """************************************************************
        ** Start process
        ************************************************************"""
        self.process()
        
    def process(self):
        if (self.algo == 'algo_dqn'):
            env = DQN()
        else:
            exit(0)
        
        global_step = 0

        for episode in range(self.load_episode+1, self.episode_size):
            global_step += 1
            local_step = 0

            state = list()
            next_state = list()
            done = False
            init = True
            score = 0

            # Reset DQN environment
            time.sleep(1.0)

            while not done:
                
                local_step += 1

                # Aciton based on the current state
                if local_step == 1:
                    action = 2  # Move forward
                else:
                    state = next_state
                    #action = int(self.get_action(state))
                    action = env.get_action(state) # action = get_action(state)

                # Send action and receive next state and reward
                req = Dqn.Request()
                print(int(action))
                req.action = action
                req.init = init
                while not self.dqn_com_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')

                future = self.dqn_com_client.call_async(req)

                while rclpy.ok():
                    rclpy.spin_once(self)
                    if future.done():
                        if future.result() is not None:
                            # Next state and reward
                            next_state = future.result().state
                            reward = future.result().reward
                            done = future.result().done
                            score += reward
                            init = False
                        else:
                            self.get_logger().error(
                                'Exception while calling service: {0}'.format(future.exception()))
                        break
                if local_step > 1:
                    env.process_next(global_step, episode, score, state, action, reward, next_state, done)

                # While loop rate
                time.sleep(0.01)

            # Update result and save model every 10 episodes
            if episode % 10 == 0:
                env.save_model()

    
def main(args=sys.argv[1]):
    rclpy.init(args=args)
    dqn_agent = Agent(args)
    rclpy.spin(agent)

    dqn_agent.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
