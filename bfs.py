n=input()
adj=[[-1 for x in range(n+1)] for y in range(4)] // north,east,south,west
for i in range(n):
    S=input()
    vec=S.split()
    id=int(vec[0])
    if(len(vec[1])>0):
        adj[id][0]=int(vec[1])
    if(len(vec[4])>0):
        adj[id][1]=int(vec[4])
    if(len(vec[2])>0):
        adj[id][2]=int(vec[2])
    if(len(vec[3])>0):
        adj[id][3]=int(vec[3])
inf=1000000000
d=[[inf for x in range(n+1)] for y in range(n+1)] 
prv=[[-1 for x in range(n+1)] for y in range(n+1)]
for i in range(1,n+1):
    d[i][i]=0
    queue=[i]
    while len(queue)>0:
        v=queue.pop(0)
        for dir in range(4):
            u=adj[v][dir]
            if(u!=-1 and d[i][v]+1<d[i][u]):
                d[i][u]=d[i][v]+1
                prv[i][u]=v
                queue.append(u)

