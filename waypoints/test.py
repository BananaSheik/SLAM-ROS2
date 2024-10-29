from machine_locations import *

cm_idx = 0
df_idx = 0
df_can_idx = 0
switch_ab = 'B'

waypoints=[]

for i in range(100):
    cm_idx = (i%36)+1

    if i%40==0:
      switch_ab = 'B' if switch_ab=='A' else 'A'
  
    df_idx=df_idx%10+1
    
    waypoints.append(carding_machines[cm_idx][0])
    waypoints.append(carding_machines[cm_idx][1])

    waypoints.append(support_points[switch_ab][df_idx])

    waypoints.append(draw_frames[switch_ab][df_idx][df_can_idx])
    waypoints.append(draw_frames[0][df_idx])

    if df_idx%10==0:
      df_can_idx+=1
      df_can_idx=df_can_idx%4
