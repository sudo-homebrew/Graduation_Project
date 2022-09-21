from tensorflow import keras
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()

import collections
from keras.layers import Activation
from keras.layers import Dense
from keras.layers import Dropout
from keras.models import Sequential
from keras.models import load_model
from keras.optimizers import RMSprop
import json
import numpy
import os
import random
import sys
import time

import rclpy
from rclpy.node import Node

from turtlebot3_msgs.srv import Dqn

class DQN():
    def __init__(self):
    
        # State size and action size
        self.state_size = 4
        self.action_size = 5
        self.episode_size = 3000

        # DQN hyperparameter
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64

        # Replay memory
        self.memory = collections.deque(maxlen=1000000)

        # Build model and target model
        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_model()
        self.update_target_model_start = 2000

        # Load saved models
        self.load_model = False
        self.load_episode = 0
        self.model_dir_path = os.path.dirname(os.path.realpath(__file__))
        self.model_dir_path = self.model_dir_path.replace(
            'turtlebot3_dqn/dqn_agent',
            'model')
        self.model_path = os.path.join(
            self.model_dir_path,
            'stage'+str(self.stage)+'_episode'+str(self.load_episode)+'.h5')

        if self.load_model:
            self.model.set_weights(load_model(self.model_path).get_weights())
            with open(os.path.join(
                    self.model_dir_path,
                    'stage'+str(self.stage)+'_episode'+str(self.load_episode)+'.json')) as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')
        

    def build_model(self):
        model = Sequential()
        model.add(Dense(
            64,
            input_shape=(self.state_size,),
            activation='relu',
            kernel_initializer='lecun_uniform'))
        model.add(Dense(64, activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dropout(0.2))
        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        model.summary()

        return model

    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    def get_action(self, state):
        if numpy.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        else:
            state = numpy.asarray(state)
            q_value = self.model.predict(state.reshape(1, len(state)))
            print(numpy.argmax(q_value[0]))
            return numpy.argmax(q_value[0])

    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def train_model(self, target_train_start=False):
        mini_batch = random.sample(self.memory, self.batch_size)
        x_batch = numpy.empty((0, self.state_size), dtype=numpy.float64)
        y_batch = numpy.empty((0, self.action_size), dtype=numpy.float64)

        for i in range(self.batch_size):
            state = numpy.asarray(mini_batch[i][0])
            action = numpy.asarray(mini_batch[i][1])
            reward = numpy.asarray(mini_batch[i][2])
            next_state = numpy.asarray(mini_batch[i][3])
            done = numpy.asarray(mini_batch[i][4])

            q_value = self.model.predict(state.reshape(1, len(state)))
            self.max_q_value = numpy.max(q_value)

            if not target_train_start:
                target_value = self.model.predict(next_state.reshape(1, len(next_state)))
            else:
                target_value = self.target_model.predict(next_state.reshape(1, len(next_state)))

            if done:
                next_q_value = reward
            else:
                next_q_value = reward + self.discount_factor * numpy.amax(target_value)

            x_batch = numpy.append(x_batch, numpy.array([state.copy()]), axis=0)

            y_sample = q_value.copy()
            y_sample[0][action] = next_q_value
            y_batch = numpy.append(y_batch, numpy.array([y_sample[0]]), axis=0)

            if done:
                x_batch = numpy.append(x_batch, numpy.array([next_state.copy()]), axis=0)
                y_batch = numpy.append(y_batch, numpy.array([[reward] * self.action_size]), axis=0)

        self.model.fit(x_batch, y_batch, batch_size=self.batch_size, epochs=1, verbose=0)
        
        
    def reset(self):
        pass
        
        
    def step(self, state):
        pass
    
    def process_next(self, global_step, episode, score, state, action, reward, next_state, done):
        # Save <s, a, r, s'> samples
        self.append_sample(state, action, reward, next_state, done)
        # Train model
        if global_step > self.update_target_model_start:
            self.train_model(True)
        elif global_step > self.train_start:
            self.train_model()
        if done:
            # Update neural network
            self.update_target_model()            
            print(
                "Episode:", episode,
                "score:", score,
                "memory length:", len(self.memory),
                "epsilon:", self.epsilon)
            param_keys = ['epsilon']
            param_values = [self.epsilon]
            param_dictionary = dict(zip(param_keys, param_values))
            
                    # Epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
    def save_model(self):
        self.model_path = os.path.join(
                    self.model_dir_path,
                    'stage'+str(self.stage)+'_episode'+str(episode)+'.h5')
        self.model.save(self.model_path)
        with open(os.path.join(
            self.model_dir_path,
                        'stage'+str(self.stage)+'_episode'+str(episode)+'.json'), 'w') as outfile:
            json.dump(param_dictionary, outfile)
        
