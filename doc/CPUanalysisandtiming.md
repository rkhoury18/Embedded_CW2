# CPU Analysis and timing

Below is a comprehensive analysis regarding timing and the way in which we use our CPU. We identify tasks performed by the system and their method of implementation. We characterise these task according to their theoretical minimum initiation interval and measured maximum execution time (The maximum execution time for each thread can be evaluated by uncommenting the DISABLE_THREADS definition, the Relevant thread or ISR to test and whether the module should be configured as a reciever or sender.). Additionally we perform critical instant analysis of the rate monotonic schedule and also calculate our CPU utilization in both sending and receiving states. Lastly we identify shared data structures and methods used to guarantee safe access and synchronisation and analyse inter-task blocking dependencies and possibility of deadlock. 

## Receiver: 

| Thread Name | Minimum Initiation Time (ms) | Maximum Execution Time ($$\mu s$$) | Task | Priority | 
| ------ | ------ | ------ | ------ | ------ |
| Scan Keys Sender | 20 |  0.0971 | Thread  | 5 |
| Display Update Reciever  | 100 |  19.91471875 | Thread  | 1 |
| CAN_TX_TASK  | 60 |  0.396 | Thread  | 2 |
| CAN_TX_ISR  |  0.7 |  0.001 | Interrupt  | NA |
| Decode Reciever  |  25.2 |  0.101 | Thread  | 4 |
| CAN_RX_ISR  |  0.7 |  0.003125 | Interrupt  | NA |
| Sample_ISR  |  0.0455 |  0.02 | Interrupt  | NA |
| Send Sound Task | 30 | 0.00371875 | Thread | 3 |
| Sample Generation | 5.8| 1.458 | Thread | 6 |

- **The critical Instance Analysis margin (ms)** = 7.55503125

- **CPU Utilization (%)** = 91.2%


## Sender: 

| Thread Name | Minimum Initiation Time (ms) | Maximum Execution Time ($$\mu s$$) | Task | Priority | 
| ------ | ------ | ------ | ------ | ------ |
| Scan Keys Sender | 20 |  0.0973 | Thread  | 5 |
| Display Update Sender  | 100 |  15.8296875 | Thread  | 1 |
| CAN_TX_TASK  | 60 |  0.396 | Thread  | 2 |
| CAN_TX_ISR  |  0.7 |  0.001 | Interrupt  | NA |
| Decode Sender  |  25.2 |  1.46 | Thread  | 4 |
| CAN_RX_ISR  |  0.7 |  0.003125 | Interrupt  | NA |
| Sample_ISR  |  0.0455 |  0.02 | Interrupt  | NA |
| Send Sound Task | 30 | 0.00371875 | Thread | 3 |
| Sample Generation task | 5.8| 1.458 | Thread | 6 |

- **The critical Instance Analysis margin (ms)** = 7.55503125

- **CPU Utilization (%)** =  92.5%





### Identification of shared data structures and methods used to guarantee safe accessand synchronisation.

Data structures that are accessed between threads executing at the same time are mainly varibales and arrays, however there are also more complicated data structures such as the double-buffer and the CAN Output Mailbox.

Variables accessed between threads are ensured to accessed safley through the use of atomic stores. Each thread can modify local copies of variables, once the thread is done modifying the variable it updates the global value using an atomic store. This guarantess no other thread can access this varible while it is being stored.

To access arrays between threads we create mutexes. If a thread 'takes' a mutex no other thread can take the mutex. Every time arrays shared between threads are accessed (read or write), the thread attempts to take the mutex. If the mutex is currently taken the thread must wait until the mutex becomes available. Once a thread is done accessing the values within the global array it 'gives' the mutex permitting other threads to continue.

Other shared values are made safe using Semaphores. Semaphores act very simalir to mutex's; if a thread is accessing them other threads must wait until they become available. The double-buffer uses a binary Semaphores to ensure thread safety between Sample-ISR and the Sample generation task, ISR_task(). The CAN_Mailbox uses a counting Semaphore of Size 3, locking access to the Semaphore once the Sempaphore has been taken 3 times. The Sempahore can be taken 3 times in this case as the CAN Mailbox has 3 slots to write values, after these are full CAN_TX_TASK() must wait for ISR to send one of the messages freeing up the mailbox, allowing the Semaphore to be taken again.

 
### Analysis of inter-task blocking dependencies and possibility of deadlock

The only tasks that have inter thread dependencies are the Sample_ISR() thread, and the ISR_Task(). ISR_Task will block until all the samples in the other half of the buffer are consumed by the interrupt. However this will not cause deadlock, as the Sample_ISR() interrupt will execute regardless of the ISR_Task() state.


