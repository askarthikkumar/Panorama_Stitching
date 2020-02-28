import argparse
import logging
import numpy as np
from random import seed
from timeit import default_timer as timer
import pygame
import gym
import gym_racer
import os
import memory
import network
from settings import *
from IPython import embed
import torch 
from graphviz import Digraph
from torchviz import make_dot
from torch.utils.tensorboard import SummaryWriter
import random
import math

class World(object):
    def __init__(self, params, env = "CartPole-v0"):
        # private variables with read access
        self.__agent = gym.make(env)
        self.__obs = self.__agent.reset()
        self.__done = False
        self.__reward = None
        self.__network = network.neural_network(self.agent.observation_space.shape[0], self.agent.action_space.n)
        self.__offline_network = network.neural_network(self.agent.observation_space.shape[0], self.agent.action_space.n, requires_grad=False)
        self.__memory = None
        # public variables
        self.render_mode = params["render_mode"]
        return

    def execute_action(self, action):
        # info is diagnostic information
        self.__obs, self.__reward, self.__done, _ = agent.step(action)
        return self.__obs, self.__reward, self.__done

    @property
    def obs(self):
        return self.__obs
    
    @property
    def done(self):
        return self.__done
    
    @property
    def agent(self):
        return self.__agent

    @property
    def reward(self):
        return self.__reward
    
    @property
    def memory(self):
        return self.__memory
    
    @property
    def network(self):
        return self.__offline_network

    def start_episode(self, x):
        action = self.__agent.action_space.sample()
        self.execute_action(action)
        return
    
    def reset_environment(self):
        self.__obs = self.__agent.reset()

    def create_database(self,thresh):
        self.__memory = memory.Memory(DATABASE_SIZE)
        self.reset_environment()
        count_net = 0;
        count_rand = 0;
        steps = 0;
        for i in range(DATABASE_SIZE):
            # if (i*100/DATABASE_SIZE)%10 == 0:
            #     print("{}% completed".format(i*100/DATABASE_SIZE))
            if(self.__done): 
                self.reset_environment()
                steps=0
            current_state_tensor = torch.from_numpy(self.obs).float().to(DEVICE)
            eps = MIN_EPSILON + (MAX_EPSILON - MIN_EPSILON) * math.exp(-LAMBDA * steps)
            if random.random()>eps:
                count_net+=1
                action = self.network(current_state_tensor)
                next_obs, reward, done = self.execute_action(torch.argmax(action,dim=0).item())
                reward = reward if not done else -reward
                self.__memory.append([self.__obs, torch.argmax(action,dim=0).item(), next_obs, reward])
            else:
                count_rand+=1;
                action = self.agent.action_space.sample()
                next_obs, reward, done = self.execute_action(action)
                reward = reward if not done else -reward
                self.__memory.append([self.__obs, action, next_obs, reward])
            # only include non-terminal states
            # assuming game is never ending
        # print("Random count is {} and Network count is {}".format(count_rand, count_net))
        self.__memory.save_database()
    
    def create_batch(self, batch):
        current_state = []
        next_state = []
        reward = []
        action = []
        for i in range(BATCH_SIZE):
            current_state.append(batch[i][0])
            action.append(batch[i][1])
            next_state.append(batch[i][2])
            reward.append(batch[i][3])
        current_state = np.vstack(current_state).astype("float")
        next_state = np.vstack(next_state).astype("float")
        action = np.array(action).astype("int")
        reward = np.array(reward).astype("float")
        return current_state, action, next_state, reward

    def map_action(index):
        # [zero,zero] not in my action space
        action_map = {
            0:[0,2], # right
            1:[0,1], # left
            2:[1,0], # up
            3:[2,0], # down
            4:[1,2], # up-right
            5:[1,1], # up-left
            6:[2,2], # down-right
            7:[2,1] #down-left
        }
        return action_map[index]
    
    def rmap_action(self, index):
        # [zero,zero] has been mapped to up
        action_map = {
            2:0, # right
            1:1, # left
            10:2, # up
            20:3, # down
            12:4, # up-right
            11:5, # up-left
            22:6, # down-right
            21:7, #down-left
            0:2
        }
        return action_map[10*index[0]+index[1]]

    def train(self):
        loss_hist = []
        agent = self.__agent
        
        writer = SummaryWriter("../Logs/")
        thresh = 1;
        for i in range(0,10):
            thresh = thresh-thresh*0.1
            for epoch in range(0, NUM_EPOCHS):
                self.create_database(thresh)
                if((epoch*100/NUM_EPOCHS)%10==0):
                    print("{}% training done".format(epoch*100/NUM_EPOCHS))
                batch = self.memory.sample(BATCH_SIZE)
                current_state, action, next_state, reward = self.create_batch(batch)
                actions_mapped = [action[i] for i in range(BATCH_SIZE)]
                action_mask = np.zeros((BATCH_SIZE, self.agent.action_space.n))
                action_mask[np.arange(BATCH_SIZE), actions_mapped] = 1
                # move arrays to device
                current_state_tensor = torch.from_numpy(current_state).float().to(DEVICE)
                next_state_tensor = torch.from_numpy(next_state).float().to(DEVICE)
                action_tensor = torch.from_numpy(action).float().to(DEVICE)
                reward_tensor = torch.from_numpy(reward).float().to(DEVICE)
                output = self.__network(current_state_tensor)
                action_mask_tensor = torch.tensor(action_mask).float()
                pred = torch.sum(torch.mul(output, action_mask_tensor),1)
                # TD Target
                next_q = self.__offline_network(next_state_tensor)
                max_q_vals, _ = torch.max(next_q, 1)
                max_q_vals = max_q_vals.view(-1)
                target = (reward_tensor + 0.95*max_q_vals).view(-1)
                # TD difference update
                loss = self.__network.loss_fn(pred, target)
                writer.add_scalar('Loss/train', loss, epoch)
                # if epoch == 0:
                #     make_dot(loss).view()
                self.__network.optimiser.zero_grad()
                loss.backward()
                self.__network.optimiser.step()
                if epoch%500 == 0 or epoch == NUM_EPOCHS-1:
                    self.__offline_network.load_state_dict(self.__network.state_dict())

            network = world.network
            network.eval()
            clock = pygame.time.Clock()
            world.reset_environment()
            while(not world.done):
                obs_tensor = torch.from_numpy(world.obs).float().to(DEVICE)
                action = network(obs_tensor)
                print(action)
                world.execute_action(torch.argmax(action,axis=0).item())
                agent.render()
                #this is necessary to hand over all OS events to pygame to be handles.
                #without this following command the pygame will not render frames
                # pygame.event.get()
                clock.tick(FPS)
        writer.close()
        torch.save(self.__offline_network.state_dict(), "trained_network")
if __name__ == "__main__":
    parser  = argparse.ArgumentParser()
    parser.add_argument("-mode", type = str, help = "\"train\" :t \"watch random policy\" : r")
    args = parser.parse_args()
    '''
    params
    params.render_mode
    params.sensor_type
    params.sensor_array_params
    '''
    params = {
        "render_mode" : RENDER_MODE,
        "sensor_type" : SENSOR_TYPE,
        "sensor_array_params" : None
    }

    world = World(params)
    agent = world.agent
    if(args.mode =='r'):
        # environment loop
        # world.network.load_state_dict(torch.load("trained_network"))
        # network = world.network.double()
        # network.eval()
        clock = pygame.time.Clock()
        world.reset_environment()
        while(True):
            obs_tensor = torch.from_numpy(world.obs).to(DEVICE)
            # action = network(obs_tensor)
            action = agent.action_space.sample()
            # world.execute_action(torch.argmax(action,axis=0).item())
            world.execute_action(action)
            print(action,world.reward, world.done)
            agent.render()
            #this is necessary to hand over all OS events to pygame to be handles.
            #without this following command the pygame will not render frames
            # pygame.event.get()
            clock.tick(FPS)
    
    if(args.mode == 't'):
        world.train()
    
