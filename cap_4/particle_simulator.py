import matplotlib.pyplot as plt
from matplotlib import animation
from random import uniform

class Particle:
    def __init__(self, x, y, ang_vel):
        self.x = x
        self.y = y
        self.ang_vel = ang_vel

class ParticleSimulator:
    def __init__(self, particles):
        self.particles = particles
        
    def evolve(self, dt):
        timestep = 0.00001
        nsteps   = int(dt/timestep)

        for i in range(nsteps):
            for p in self.particles:
                norm = (p.x**2 + p.y**2)**0.5
                v_x = -p.ang_vel * p.y / norm * timestep
                v_y = p.ang_vel * p.x / norm * timestep
                p.x += v_x
                p.y += v_y
def visualize(simulator):

    X = [p.x for p in simulator.particles]
    Y = [p.y for p in simulator.particles]
    
    fig = plt.figure()
    ax  = plt.subplot(111,aspect = 'equal')
    line, = ax.plot(X, Y, 'ro')

    plt.xlim(-1,1)
    plt.ylim(-1,1)
    
    def init():
        line.set_data([],[])
        return line,

    def animate(i):
        simulator.evolve(0.01)
        X = [p.x for p in simulator.particles]
        Y = [p.y for p in simulator.particles]
        line.set_data(X,Y)
        return line,

    anim = animation.FuncAnimation(fig,
                                   animate,
                                   init_func = init,
                                   blit = True,
                                   interval = 10)
    plt.show()

def test_visualize():
    particles = [Particle(0.3,0.5,1),
                 Particle(0.0,-0.5,-1),
                 Particle(-0.1,-0.4,3)]
    simulator = ParticleSimulator(particles)
    visualize(simulator)


def test_evolve():
    particles = [Particle(0.3,0.5,1),
                 Particle(0.0,-0.5,-1),
                 Particle(-0.1,-0.4,3)]
    simulator = ParticleSimulator(particles)
    simulator.evolve(0.1)
    
    p0, p1, p2 = particles
    def fequal(a,b,eps=1e-5):
        return abs(a-b) < eps
    assert fequal(p0.x, 0.210269)
    assert fequal(p0.y, 0.543863)
    
    assert fequal(p1.x, -0.099334)
    assert fequal(p1.y, -0.490034)
    
    assert fequal(p2.x, 0.191358)
    assert fequal(p2.y, -0.365227)
    
    
def benchmark():
    particles = [Particle(uniform(-1,1), uniform(-1,1), uniform(-1,1)) for i in range(1000)]
    simulator = ParticleSimulator(particles)
    simulator.evolve(0.1)

'''
        result = timeit.timeit('benchmark()',
                           setup = 'from __main__ import benchmark', 
                           number = 10)
    
    result = timeit.repeat('benchmark()',
                           setup = 'from __main__ import benchmark', 
                           number = 10,
                           repeat = 3)
    mean = np.mean(np.array(result/10))
    std = np.std(np.array(result/10))
    print("T = {} \u00B12 {} s".format(mean,std))
'''
    
if __name__ == "__main__":
    
    test_visualize()
    test_evolve()
    benchmark()