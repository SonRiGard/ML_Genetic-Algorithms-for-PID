# ML_Genetic-Algorithms-for-PID

Before you continue, ensure you meet the following requirements:

* You have installed the version of Python 3.
* You have a basic understanding PID theory.

# Steps

1. Import neccessary library

```sh
    import random
    import serial
    import time
```

2. Open serial port in this example use in windown

```sh
    ser = serial.Serial(port='COM5', baudrate=115200, timeout=1)
```

3. Define the range of values for the gene

```sh
    k=[0.6,0.6,0.18] #kp,ki,kd được tính toán từ Phương pháp Ziegler
    alpha=0.1 # hệ số biên dưới 
    beta=10 # hệ số biên trên
    K_IAE = 1 # trọng số trong phương trình đánh giá khả năng PID
    K_IATE = 1
    k_MSE = 1
```

4. Create a random gene with values between ki*ampha and k[i]*betal

```sh
    def create_individual():
        return [random.uniform(k[i]*alpha, k[i]*beta) for i in range(3)]
```

5. Create initial population

```sh
    def create_population(pop_size):
        return [create_individual() for _ in range(pop_size)]
```

6. Fitness function - minimize the error

```sh
    def fitness(individual):
        #---------------------------------------------
        Kp, Ki, Kd = individual
        start_time = sent_Kpid(Kp,Ki,Kd)
        #Lệnh vòng lặp điều kiện cho đến khi kết thúc 
        #Đợi cho đến khi nhận tín hiệu start từ vdk(để vdk chuẩn bị cho quá trình test pid cho hệ số này)
        result = simulate_pid(3,start_time,K_IAE,K_IATE,k_MSE)
        return result
```

7. Send new PID in each individual

```sh
    def sent_Kpid (ukp,uki,ukd):
        time.sleep(2)
        print("seding kp : ...")
        ser.write(bytes(str(ukp)+"\n", 'utf-8'))
        time.sleep(0.1)
        print("sent kp!")
        
        print("seding kI : ...")
        ser.write(bytes(str(uki)+"\n", 'utf-8'))
        time.sleep(0.1)
        print("sent ki!")
            
        print("seding kd : ...")
        ser.write(bytes(str(ukd)+"\n", 'utf-8'))
        print("sent kp!")
        time.sleep(0.1)

        ACK_start_cur_pid=["",""]
        while (ACK_start_cur_pid[1] != "111"):
            ACK_start_cur_pid =  ser.readline().decode().rstrip().split(",")
        print(ACK_start_cur_pid)
        return float(ACK_start_cur_pid[0]) 
        #return start time of current process for next step
```

8. Run the test and return an individual's results    

```sh
    def simulate_pid (N_time_out,time_start,a,b,c):#a,b,c : scale factor of functions IAE,IATE,MSE
        t=0
        J1=J2=J3=J=0
        t = time_start
        N=0
        Time_out = 0
        while (1):
        # ser.reset_input_buffer()
            N+=1
            Rx_data=ser.readline()
            if Rx_data == '':
                Time_out += 1
            print(Rx_data)
            # print(data[0]+data[1]+data[2])
            data = Rx_data.decode().rstrip().split(',')
            pre_time=t
            t=float(data[1])-time_start
            dt=t-pre_time
            if Time_out < N_time_out :
                if len(data) == 3:
                    J1+=abs(float(data[0]))*dt#IAE
                    J2+=t*abs(float(data[0]))*dt#IATE
                    J3+=1/N*(float(data[0])*float(data[0]))#MSE
                    J+=a*J1+b*J2+c*J3        
                    if data[2] == "222":#ACk stop process
                        break#end of while()
            else :
                break
        return J1,J2,J3,J
```

9. Selection function - tournament selection (Tạo giải đấu giữu các cá nhân lấy ra k số lượng cá nhân)

```sh
    def tournament_selection(population, k):
        selected = []
        for i in range(k):
            tournament = random.sample(population, 2)
            fitness1 = fitness(tournament[0])
            fitness2 = fitness(tournament[1])
            if fitness1 < fitness2:
                selected.append(tournament[0])
            else:
                selected.append(tournament[1])
        return selected
```

10. Crossover function - single point crossover : (tái tổ hợp)
    Wiki: Single Point Crossover: A crossover point on the parent organism string is selected. All data beyond that point in the organism string is swapped between the two parent organisms. Strings are characterized by Positional Bias.

```sh
    def single_point_crossover(parents):
        point = random.randint(0, len(parents[0])-1)
        child1 = parents[0][:point] + parents[1][point:]
        child2 = parents[1][:point] + parents[0][point:]
        return child1, child2
```

11. Mutation function - uniform mutation (Đột biến)

```sh
    def uniform_mutation(individual):
        gene_index = random.randint(0, len(individual)-1)
        individual[gene_index] = random.uniform(k[gene_index]*alpha,k[gene_index]*beta)
        return individual
```

12. Genetic algorithm

```sh
    def genetic_algorithm(target, pop_size, k, num_generations, mutation_prob):
        # create initial population
        population = create_population(pop_size)
        
        # run evolution for num_generations
        for gen in range(num_generations):
            # perform tournament selection
            parents = tournament_selection(population, k, target)
            #Tạo giải đấu và tạo ra K cặp bố mẹ

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
                return population[0][3], gen

        # return best individual and number of generations run
        return population[0], num_generations
```
