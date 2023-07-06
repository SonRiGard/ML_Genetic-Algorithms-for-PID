import random
import serial

# Open serial port
ser = serial.Serial('COM9', 115200)
# Close serial port
ser.close()
# define the range of values for the gene
k=[0.6,0.6,0.18] #kp,ki,kd được tính toán từ Phương pháp Ziegler
alpha=0.1#xem dinh nghia ampha betal trong file word giới thiệu
beta=10

# create a random gene with values between ki*ampha and k[i]*betal
def create_individual():
    return [random.uniform(k[i]*alpha, k[i]*beta) for i in range(3)]

# create initial population
def create_population(pop_size):
    return [create_individual() for _ in range(pop_size)]

# fitness function - minimize the error
def fitness(individual, target):
    # thêm lệnh để gởi đến vi điều khiển để (đồng bộ với GA) bắt đầu quá trình chạy test PID 
    #---------------------------------------------
    Kp, Ki, Kd = individual
    ser.writelines(str(Kp)+","+str(Ki)+","+str(Kd))
    J1=0
    J2=0
    J3=0
    N=1
    #Lệnh vòng lặp điều kiện cho đến khi kết thúc 
    #Đợi cho đến khi nhận tín hiệu start từ vdk(để vdk chuẩn bị cho quá trình test pid cho hệ số này)
    while ser.readline().strip() == b'start':
        pass
    error=t=dt=J1=J2=J3=0

    #cần thêm hàm try nếu serial lỗi và nhận kí tự sai(không phải số)
    #
    #Vòng lặp tính lỗi theo thời gian
    while ser.readline().strip() != 'stop' :
    # Read data from serial port data[0] : error; data[1] : time; data[2] : dt
        data_read_serial = ser.readline() # Read one line of data from serial port

        data=data_read_serial.split(",")

        error=float(data[0].decode().strip())
        t = float(data[1].decode().strip())
        dt= float(data[2].decode().strip())

        J1=abs(error)*dt#IAE
        J2=t*abs(error)*dt#IATE
        J3=1/N*(error*error)#MSE
        #có thể thay đổi trọng số của các J1, J2, J3 bằng cách nhân với một hệ số nhất định
        # ex : J=0.3*J1+0.3*J2+0.1*J3
        J=J1+J2+J3
    return J

# selection function - tournament selection
def tournament_selection(population, k, target):
    selected = []
    for i in range(k):
        tournament = random.sample(population, 2)
        fitness1 = fitness(tournament[0], target)
        fitness2 = fitness(tournament[1], target)
        if fitness1 < fitness2:
            selected.append(tournament[0])
        else:
            selected.append(tournament[1])
    return selected

# crossover function - single point crossover
def single_point_crossover(parents):
    point = random.randint(0, len(parents[0])-1)
    child1 = parents[0][:point] + parents[1][point:]
    child2 = parents[1][:point] + parents[0][point:]
    return child1, child2

# mutation function - uniform mutation
def uniform_mutation(individual):
    gene_index = random.randint(0, len(individual)-1)
    individual[gene_index] = random.uniform(k[gene_index]*alpha,k[gene_index]*beta)
    return individual

# simulate PID controller and return the error
def simulate_pid(Kp, Ki, Kd):
    # TODO: implement PID controller simulation
    return 0

# genetic algorithm
def genetic_algorithm(target, pop_size, k, num_generations, mutation_prob):
    # create initial population
    population = create_population(pop_size)
    
    # run evolution for num_generations
    for gen in range(num_generations):
        # perform tournament selection
        parents = tournament_selection(population, k, target)

        # perform crossover
        children = []
        for i in range(0, len(parents), 2):
            child1, child2 = single_point_crossover([parents[i], parents[i+1]])
            children.append(child1)
            children.append(child2)

        # perform mutation
        for i in range(len(children)):
            if random.random() < mutation_prob:
                children[i] = uniform_mutation(children[i])

        # create new population
        population = parents + children

        # sort population by fitness
        population.sort(key=lambda x: fitness(x, target))

        # truncate population to original size
        population = population[:pop_size]

        # check for perfect solution
        best_fitness = fitness(population[0], target)
        if best_fitness == 0:
            return population[0], gen

    # return best individual and number of generations run
    return population[0], num_generations

# example usage for genetic algorithm
TARGET = 0 # target error
POPULATION_SIZE = 100
K = 5 # tournament selection size
NUM_GENERATIONS = 50
MUTATION_PROB = 0.05
best_individual, num_generations = genetic_algorithm(TARGET, POPULATION_SIZE, K, NUM_GENERATIONS, MUTATION_PROB)
print("The best individual is: ", best_individual)
print("Number of generations: ", num_generations)