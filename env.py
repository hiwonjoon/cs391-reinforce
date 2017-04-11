# -*- coding: utf-8 -*-
import better_exceptions
import random
import numpy as np
import math
from asciimatics.screen import Screen

class World(object) :
    line_alloc = [1,2,2,1,1] #side_walk, road, road, side_walk, traffic_light
    line_range = [[0],[1,2],[3,4],[5],[6]]
    road_length=100
    """
    _t = 0
    _range = [] #for coordinate transform(due to line drawing)
    for line in self.line_alloc :
        _range.append(range(_t,_t+line))
        _t += line
    """
    def __init__(self,time_step=1.0/100) :
        self.global_time = 0.0
        self.time_step = time_step
        self.objects = []

    def add_obj(self,obj) :
        assert(isinstance(obj,GameObject))
        self.objects.append(obj)
        return self

    def tick(self) :
        # Tick
        for obj in self.objects: obj.tick(self.time_step)
        # Interaction
        for obj_a in self.objects :
            for obj_b in self.objects :
                if( obj_a == obj_b ) : continue
                obj_a.interact(obj_b,self.time_step)
        # Constraint Check(Interaction with wall..) or Delteable Object
        for obj in self.objects: pass #TODO: Nothing to do for now.
        self.objects = [obj for obj in self.objects if not obj.remove]
        self.global_time += self.time_step

    def draw_lines(self,screen) : # Let's draw lines/one time call
        _t = 0
        for line,c in zip(self.line_alloc[:-1],[7,3,7,1,4]) :
            screen.move(0,_t+line)
            screen.draw(self.road_length,_t+line,thin=True,colour=c)
            _t += line+1

    def draw(self,screen) :
        current_map = np.empty((self.road_length,sum(self.line_alloc)),dtype=object)
        for obj in self.objects: current_map[obj.loc[0],obj.loc[1]] = obj

        # Let's draw objects
        for x in range(self.road_length) :
            for y in range(sum(self.line_alloc)) :
                if( current_map[x,y] is None ) :
                    screen.print_at(' ', x,y+[k for k,_t in enumerate(self.line_range) if y in _t][-1] )
                else :
                    screen.print_at( current_map[x,y].char(), x,y+[k for k,_t in enumerate(self.line_range) if y in _t][-1],
                                    **current_map[x,y].repre() )

        # Debug Info
        screen.print_at('%3.3f Secs'%(self.global_time), self.road_length+1,0)
        #msg = '%s'%(current_map)
        #screen.print_at(msg,0,sum(self.line_alloc)+len(self.line_alloc)+1)

class GameObject(object) :
    def __init__(self,x,y) :
        self.loc = np.array((x,y),np.int32)
        self.remove = False
    def tick(self,delta) :
        pass
    def char(self) :
        return '*'
    def repre(self) :
        return {}
    def interact(self,other,delta) :
        # The result of interaction only affect the "self".
        pass
    def __repr__(self):
        return '%s(%d,%d)'%(type(self).__name__,self.loc[0],self.loc[1])

class TrafficLights(GameObject) :
    light_color = [3,1,2]
    def __init__(self,x,y=6,time_schedule=[2,5,5]):
        GameObject.__init__(self,x,y)
        self.state = 0
        self.time = 0.0
        self.time_schedule=time_schedule
    def tick(self,delta) :
        self.time += delta
        if( self.time >= self.time_schedule[self.state] ) :
            self.time = 0.0
            self.state = (self.state+1)%len(self.time_schedule)
    def get_state(self) :
        if(self.state == 0) : return 'yellow'
        elif(self.state == 1 ) : return 'red'
        else : return 'green'
    def char(self) :
        return u'●'
        #return u'0'
    def repre(self) :
        return {'colour':self.light_color[self.state]}

    def _is_crossing(self,car,delta) :
        dist_a = (self.loc - car.real_loc)[0]
        dist_b = (self.loc - car._predict_loc(delta))[0]
        return dist_a * dist_b <= 0

    def interact(self,other,delta) :
        if (isinstance(other,Car)):
            if( self.get_state() == 'red'
               and self._is_crossing(other,delta) ):
                pass
                #assert False,'Ticket!'

class Movable(GameObject) :
    def __init__(self,x,y):
        GameObject.__init__(self,x,y)

class Car(Movable) :
    def __init__(self,x=None,y=None,v=None):
        x = x or 0
        y = y or random.randint(World.line_range[1][0],World.line_range[2][-1])
        v = float(v or random.randint(1,3))
        Movable.__init__(self,x,y)

        self.real_loc = self.loc.astype(np.float32)
        self.maximum_vel = v
        self.vel = v
        self.direction = -1 if y in World.line_range[1] else 1
        self.state = 'go' #'go', 'stop', 'park'
        self.constraint_queue = [] #if it is empty, state change to 'go'
    def char(self) :
        if( self.state == 'park' ) :
            return 'P'
        elif( self.vel == 0.0 ) :
            return 'S' #completely stopped
        elif( self.state == 'go' ) :
            return u'◀' if( self.direction < 0 ) else u'▶'
        elif( self.state == 'stop' ) : #stopping...
            return u'↤' if( self.direction < 0 ) else u'↦'
        else :
            assert(False)

    def tick(self,delta) :
        # Update Speed
        if( self.state == 'go' ) :
            self.vel += delta * 1.0
        elif( self.state == 'stop' ) :
            self.vel += delta * -self.maximum_vel
        self.vel = max(min(self.maximum_vel, self.vel),0.0)

        # Update Location
        self.real_loc[0] += delta * self.vel * self.direction
        if( self.real_loc[0] < 0 ) : self.real_loc[0] += World.road_length
        elif( self.real_loc[0] > World.road_length ) : self.real_loc[0] -= World.road_length

        self.loc = self.real_loc.astype(np.int32)

    def _predict_loc(self,future_secs) :
        pre = np.copy(self.real_loc)
        #pre[0] += future_secs * self.vel * self.direction
        pre[0] += future_secs * self.maximum_vel * self.direction
        return pre

    def _is_in_front(self,car) : #Does I in front of other car?
        d = self._dist(car)
        if( self.real_loc[0] > car.real_loc[0] and
            self.real_loc[0] - car.real_loc[0] == d ) :
            return True if self.direction > 0 else False
        elif( self.real_loc[0] > car.real_loc[0] ) :
            return False if self.direction > 0 else True
        elif( self.real_loc[0] < car.real_loc[0] and
             car.real_loc[0] - self.real_loc[0] == d ) :
            return False if self.direction > 0 else True
        else :
            return True if self.direction > 0 else False

    def _dist(self,car) :
        # choose the short one as dist; due to cycling behavior
        _t = abs(car.real_loc[0] - self.real_loc[0])
        return min(_t, World.road_length-_t)

    def interact(self,other,delta) :
        if( len(self.constraint_queue) > 0 ) : # means, it is in the stop state.
            self.constraint_queue = [q for q in self.constraint_queue if q(other) == False]
            if( len(self.constraint_queue) == 0 ):
                self.state = 'go'

        # interact with other cars and pedestrians and traffic lights.
        if (isinstance(other,TrafficLights)) :
            if( self.state == 'go') :
                dist_a = (other.loc - self.real_loc)[0]
                dist_b = (other.loc - self._predict_loc(2.0))[0]
                crossing = dist_a * dist_b <= 0
                if( crossing ) :
                    if( other.get_state() == 'red' or
                        other.get_state() == 'yellow' ) :
                        self.state = 'stop'
                        self.constraint_queue.append(lambda o: other == o and other.get_state() == 'green')
        elif (isinstance(other,Car)):
            if( (other.loc == self.loc).all() ) :
                assert False,'Car Crash!'
            if( other.loc[1] == self.loc[1]) : # On the same lane
                if( other._is_in_front(self) ) : #if other car is in front of me/TODO: since the road is cycle, it is not perfect.
                    # Check the safety distance
                    if( self._dist(other) < 3 and other.vel - self.vel <= 0) :
                        self.state = 'stop'
                        self.constraint_queue.append(lambda o: other == o and self._dist(other) > 2)
        elif (isinstance(other,Pedestrian)):
            dist_a = (other.loc - self.real_loc)[0]
            dist_b = (other.loc - self._predict_loc(2.0))[0]
            crossing = dist_a * dist_b < 0
            if( crossing ) :
                self.state = 'stop'
                self.constraint_queue.append(lambda o: other.remove)

class MyCar(Car) :
    def __init__(self,x,y,v):
        Car.__init__(self,x,y,v)
    def repre(self) :
        return {'colour':1} #my car is red colored!

    #############
    # Functions for reinforcement learning
    #############
    def interact(self,other,delta) :
        # this function will be called every tick(fine-grained time steps)
        # TODO: encode the state for timestep(coarse-grained), and accumulate
        # reward for that time.
        return NotImplemented
    def get_state_and_reward(self,world) :
        # This function should be called by reinforcement module(like SARSA
        # algorithm) outside for every timestep.
        return NotImplemented
    def set_action(self,action) :
        # This function will be called by reinforcement module(like SARSA
        # algorithm) outside for every time-step. Decide the next action for
        # next timestep.
        assert action == 'go' or action == 'stop'
        self.state = action

class Pedestrian(Movable) :
    def __init__(self,x=None,y=None,time_schedule=[1,999,1]): #cross_speed=1
        x = x or random.randint(10,World.road_length-10)
        y = y or World.line_range[0][-1] if random.randint(0,1) == 0 else World.line_range[3][-1]
        Movable.__init__(self,x,y)

        self.state = 0
        self.time = 0.0
        self.time_schedule = time_schedule
        self.goal = World.line_range[3][-1] if y == World.line_range[0][-1] else World.line_range[0][-1]
        self.direction = -1 if y > self.goal else 1

    def char(self) : return u'☺'
    def tick(self,delta) :
        if( self.state >= len(self.time_schedule) ) :
            self.remove = True
            return

        self.time += delta
        if( self.time >= self.time_schedule[self.state] or self.loc[1] == self.goal ) :
            self.time = 0.0
            self.state += 1
        elif( self.state == 1 and self.time >= 1.0 ) :
            self.time = 0.0
            self.loc[1] += self.direction

    def _is_squashing_me(self,car,delta) :
        dist_a = (self.loc - car.real_loc)[0]
        dist_b = (self.loc - car._predict_loc(delta))[0]
        return self.loc[1] == car.real_loc[1] and dist_a * dist_b <= 0

    def interact(self,other,delta) :
        if (isinstance(other,Car) and
            self._is_squashing_me(other,delta) ):
            pass
            #assert False,'I am dead :('

if __name__ == "__main__":
    world = World()
    world.add_obj(TrafficLights(x=10,time_schedule=[1,1,2]))
    #world.add_obj(TrafficLights(x=10))
    world.add_obj(TrafficLights(x=70,time_schedule=[1,4,4]))
    def main_loop(screen) :
        def _seconds(s,continuous=True) :
            time = 0.0
            while(time <= s) :
                world.tick();world.draw(screen);
                if(continuous) : screen.refresh()
                time += world.time_step
            screen.refresh();
        world.draw_lines(screen)
        go = True
        while(True) :
            ev = screen.get_key()
            if ev in [ord('q')] :
                return
            elif ev in [ord('s')] and not go:
                _seconds(1,continuous=False)
            elif ev in [ord('g')]:
                go = not go
            elif ev in [ord('p')]:
                world.add_obj(Pedestrian())
            elif ev in [ord('c')]:
                world.add_obj(Car())
            if go :
                _seconds(1)
    Screen.wrapper(main_loop,arguments=[])
    print '*****Debug Information*****'
