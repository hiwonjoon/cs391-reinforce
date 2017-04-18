# -*- coding: utf-8 -*-
import better_exceptions
import random
import numpy as np
import math
from asciimatics.screen import Screen

class ViolateRule() :
    def __init__(self,message):
        self.message = message

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

        #debug message
        self.debug_list = []
        self.message = []

    def add_obj(self,obj,debug=False) :
        assert(isinstance(obj,GameObject))
        self.objects.append(obj)
        if( debug ) :
            self.debug_list.append(obj)
        return self

    def tick(self) :
        # Tick
        for obj in self.objects:
            for e in obj.tick(self.time_step) :
                if( e is not None and obj in self.debug_list ):
                    self.message.append(e.message)
        # Interaction
        for obj_a in self.objects :
            for obj_b in self.objects :
                if( obj_a == obj_b ) : continue
                for e in obj_a.interact(obj_b,self.time_step) :
                    if( e is not None and obj_a in self.debug_list ):
                        self.message.append(e.message)

        # Constraint Check(Interaction with wall..) or Delteable Object
        for obj in self.objects: pass #TODO: Nothing to do for now.
        self.objects = [obj for obj in self.objects if not obj.remove]
        self.global_time += self.time_step
        return self.time_step

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
        for i,msg in enumerate(self.message) :
            screen.print_at(msg, self.road_length+1,i+1)
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
    def __init__(self,x,y=6,time_schedule=[2,5,5],start_time=0.0):
        GameObject.__init__(self,x,y)
        self.state = 0
        self.time = start_time
        self.time_schedule=time_schedule
    def tick(self,delta) :
        self.time += delta
        if( self.time >= self.time_schedule[self.state] ) :
            self.time = 0.0
            self.state = (self.state+1)%len(self.time_schedule)
        yield
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
                yield ViolateRule('Ticket!')
        yield

class Movable(GameObject) :
    def __init__(self,x,y):
        GameObject.__init__(self,x,y)

class Car(Movable) :
    def __init__(self,x=None,y=None,v=None,state=None):
        x = x if x is not None else 0
        y = y if y is not None else random.randint(World.line_range[1][0],World.line_range[2][-1])
        v = float(v if v is not None else random.randint(1,3))
        state = state if state is not None else 'go'
        Movable.__init__(self,x,y)

        self.real_loc = self.loc.astype(np.float32)
        self.maximum_vel = v
        self.vel = v
        self.direction = -1 if y in World.line_range[1] else 1
        self.state = state #'go', 'stop', 'park', 'left', 'right'
        self.constraint_queue = [] #if it is empty, state change to 'go'
    def char(self) :
        if( self.state == 'park' ) :
            return 'P'
        elif( self.vel == 0.0 ) :
            return 'S' #completely stopped
        elif( self.state == 'go') :
            return u'◀' if( self.direction < 0 ) else u'▶'
        elif( self.state == 'stop' ) : #stopping...
            return u'↤' if( self.direction < 0 ) else u'↦'
        elif( self.state == 'left' ):
            return u'⬋' if( self.direction < 0 ) else u'⬈'
        elif( self.state == 'right' ):
            return u'⬉' if( self.direction < 0 ) else u'⬊'
        else :
            assert(False)

    def tick(self,delta) :
        if( self.state == 'park' ) :
            yield;return

        # Update Changing Lane
        if( self.state == 'left' or self.state == 'right' ) :
            self.real_loc[1] += (self.direction * (-1 if self.state == 'left' else 1))
            self.state = 'go'

            if( self.real_loc[1] < World.line_range[1][0] or
                self.real_loc[1] > World.line_range[2][-1]) :
                yield ViolateRule('Car Off Track!%f'%self.real_loc[1])
                self.real_loc[1] = max(min(World.line_range[2][-1], self.real_loc[1]),World.line_range[1][0])

        # Update Speed
        if( self.state == 'go' ) :
            self.vel += delta * self.maximum_vel
        elif( self.state == 'stop' ) :
            self.vel += delta * -self.maximum_vel
        self.vel = max(min(self.maximum_vel, self.vel),0.0)

        # Update Location
        self.real_loc[0] += delta * self.vel * self.direction
        if( self.real_loc[0] < 0 ) : self.real_loc[0] += World.road_length
        elif( self.real_loc[0] > World.road_length ) : self.real_loc[0] -= World.road_length

        self.loc = self.real_loc.astype(np.int32)
        yield

    def _predict_loc(self,future_secs) :
        pre = np.copy(self.real_loc)
        pre[0] += future_secs * self.vel * self.direction
        return pre

    def _is_in_front(self,car) : #Does I in front of other car?
        if( self.direction != car.direction ) : return True

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
        if( self.direction != car.direction ):
            if( self.direction == 1 ) : # and it means car.direction == -1
                return abs(car.real_loc[0] - self.real_loc[0])
            else :
                return abs(self.real_loc[0] - car.real_loc[0])
        else :
            # choose the short one as dist; due to cycling behavior
            _t = abs(car.real_loc[0] - self.real_loc[0])
            return min(_t, World.road_length-_t)

    def interact(self,other,delta) :
        if( self.state == 'park' ) :
            yield;return

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
                yield ViolateRule('Car Crash!')
            if( other.loc[1] == self.loc[1]) : # On the same lane
                if( other._is_in_front(self) ) : #if other car is in front of me
                    # Check the safety distance
                    if( self._dist(other) < 3 and other.vel - self.vel <= 0) :
                        if( other.state == 'park') :
                            self.state = 'left'
                        else :
                            self.state = 'stop'
                            self.constraint_queue.append(lambda o: other == o and self._dist(other) > 2)
                elif( self._is_in_front(other) and self._dist(other) < 3 and
                      self.vel < other.vel and
                      (self.direction==1 and self.loc[1] == 3 or #TODO: Hard code warning!
                       self.direction==-1 and self.loc[1] == 2)) :
                    self.state = 'right'

        elif (isinstance(other,Pedestrian)):
            dist_a = (other.loc - self.real_loc)[0]
            dist_b = (other.loc - self._predict_loc(2.0))[0]
            crossing = dist_a * dist_b < 0
            if( crossing ) :
                self.state = 'stop'
                self.constraint_queue.append(lambda o: other.remove)
        yield

class MyCar(Car) :
    def __init__(self,x,y,v):
        Car.__init__(self,x,y,v)
        self.init_state()

    def repre(self) :
        return {'colour':1} #my car is red colored!

    #############
    # Functions for reinforcement learning
    #############
    def set_action(self,action) :
        # This function will be called by reinforcement module(like SARSA
        # algorithm) outside for every time-step. Decide the next action for
        # next timestep.
        assert action == 'go' or action == 'stop' or action == 'left' or action =='right'
        self.state = action

    def tick(self,delta) :
        for e in super(MyCar, self).tick(delta) :
            if( e is not None and 'Off' in e.message ): self._off_track.append(True)
            yield e
    def interact(self,other,delta) :
        # this function will be called every tick(fine-grained time steps)
        # encode the state for timestep(coarse-grained), and accumulate
        # accumulate information for reward.
        if (isinstance(other,TrafficLights)) :
            if( other.get_state() == 'red' and
                other._is_crossing(self,delta)
               ) :
                self._traffic_tickets.append(True)
                yield ViolateRule('Ticket!')
        elif (isinstance(other,Car)):
            if( (self.loc == other.loc).all() and
                other not in self._hit_cars) :
                self._hit_cars.add(other)
                yield ViolateRule('Car Crash!')
        elif (isinstance(other,Pedestrian)):
            if( other._is_squashing_me(self,delta) ):
                self._hit_pedestrians.append(True)
                yield ViolateRule('Hit Pedestrian!')
        yield

    def init_state(self):
        # accumulated info for calculating reward
        self._traffic_tickets = []
        self._hit_pedestrians = []
        self._hit_cars = set()
        self._off_track = []
        self._prev_loc = self.loc

    def get_state_and_reward(self,world) :
        # This function should be called by reinforcement module(like SARSA
        # algorithm) outside for every timestep.
        dist_moved = (self.loc[0] - self._prev_loc[0])*self.direction % world.road_length
        reverse_drive = False
        if ( int(self._prev_loc[1]) in World.line_range[1]) :
            reverse_drive = True

        state = self._get_state(world)
        reward =  dist_moved * (0.5 if int(self._prev_loc[1]) == World.line_range[2][1]
                               else 0.3) + \
                 (-1.5 if reverse_drive else 0.0) + \
                 (np.count_nonzero(np.array(self._traffic_tickets))*-10.0) + \
                 (np.count_nonzero(np.array(self._hit_pedestrians))*-20.0) + \
                 len(self._hit_cars)*-5.0 + \
                 len(self._off_track)*-10.0
                 #int(dist_moved == 0) * (-1.0) + \

        self.init_state()
        return state, reward

class Pedestrian(Movable) :
    def __init__(self,x=None,y=None,time_per_cell=1.0,time_schedule=[1,999,1]): #cross_speed=1
        x = x or random.randint(10,World.road_length-10)
        y = y or World.line_range[0][-1] if random.randint(0,1) == 0 else World.line_range[3][-1]
        Movable.__init__(self,x,y)

        self.state = 0
        self.time = 0.0
        self.time_per_cell = time_per_cell
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
        elif( self.state == 1 and self.time >= self.time_per_cell ) :
            self.time = 0.0
            self.loc[1] += self.direction
        yield

    def _is_squashing_me(self,car,delta) :
        dist_a = (self.loc - car.real_loc)[0]
        dist_b = (self.loc - car._predict_loc(delta))[0]
        return self.loc[1] == car.real_loc[1] and dist_a * dist_b <= 0

    def interact(self,other,delta) :
        if (isinstance(other,Car) and
            self._is_squashing_me(other,delta) ):
            yield ViolateRule('Hit Person!')
        yield

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
            elif ev in [ord('u')]:
                world.add_obj(Car(x=random.choice(range(1,99)),y=random.choice([1,4]),v=0,state='park'))
            elif ev in [ord('c')]:
                car = Car()
                world.add_obj(car,True)
            elif ev in [ord('l')]:
                car.state = 'left'
            elif ev in [ord('r')]:
                car.state = 'right'
            if go :
                _seconds(1)
    Screen.wrapper(main_loop,arguments=[])
    print '*****Debug Information*****'
