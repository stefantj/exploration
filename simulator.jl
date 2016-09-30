# Simulator module. 
using PyPlot
using JLD
using Graphs
include("../Libraries/FMT/fmt.jl")
using FMT
include("buildings.jl")
include("hungarian.jl")
using Hungarian


### Containers for agent information ###
type StateData
    map::Array{Float64,2}
    locs::Array{Float64,2}
    bearings::Array{Float64,1}
end

type Belief
    state::StateData
    fronts::Array{Int64,1}
    unexplored_list::Array{Int64,1}
end

# Contains planning data (start pt, goal pt, steps in the path)
type Plan
    value::Float64
    goal::Array{Float64,1}
    path::Array{Float64,2}
end


### Helper functions for simple tasks ###

function loc2ind(i,j,mapsize)
    return round(Int,round(Int,i) + (round(Int,j)-1)*mapsize)
end 

function ind2loc(ind,mapsize)
    # start with 0-based indexing then extend to 1-based:
    return [round(Int,mod(ind-1,mapsize))+1, floor(Int,(ind-1)/mapsize)+1]
end

# Add two uncertainty maps
function map_add(map1,map2)
    xlim = size(map1,1)
    ylim = size(map1,2)
    new_map = 0.5*ones(map1)
    for i = 1:xlim
        for j =1:ylim
            if(map1[i,j] == 1 || map2[i,j] == 1)
                new_map[i,j] = 1
            elseif(map1[i,j] == 0 || map2[i,j] == 0)
                new_map[i,j] = 0
            end
        end
    end    
    return new_map
end

# flips a map along the given dimension
function map_flip(dim, map)
    new_map = zeros(map)
    i_max = size(map,1)
    j_max = size(map,2)
    for i = 1:size(map,1)
        for j = 1:size(map,2)
            if(dim==1) # Flip i dimension
                new_map[i_max - i + 1, j] = map[i,j]
            else # Flip j dimension
                new_map[i, j_max - j + 1] = map[i,j]
            end
        end
    end
    return new_map
end

# concactenates 4 maps:
# 1 2
# 3 4
function concat_4_maps(map1,map2,map3,map4)
    i_max1 = size(map1,1); j_max1 = size(map1,2)
    i_max2 = size(map2,1); j_max2 = size(map2,2)
    i_max3 = size(map3,1); j_max3 = size(map3,2)
    i_max4 = size(map4,1); j_max4 = size(map4,2)
    
    # check for consistency. This could be relaxed.
    if(i_max1 != i_max3)
        println("Error: incompatible first dimension for map 1, 3")
        return []
    elseif(i_max2 != i_max4)
        println("Error: incompatible first dimension for map 2, 4")
        return []
    elseif(j_max1 != j_max2)
        println("Error: incompatible second dimension for map 1, 2")
        return []
    elseif(j_max3 != j_max4)
        println("Error: incompatible second dimension for map 3, 4")
        return []
    end

    new_map = zeros(i_max1+i_max2, j_max1+j_max3)
    for i = 1:i_max1
        for j = 1:j_max1
            new_map[i,j] = map1[i,j]
        end
        for j = 1:j_max3
            new_map[i, j_max1+j] = map3[i,j]
        end
    end
    for i = 1:i_max2
        for j = 1:j_max2
            new_map[i+i_max1,j] = map2[i, j]
        end
        for j = 1:j_max4
            new_map[i+i_max1,j+j_max2] = map4[i,j] 
        end
    end

    return new_map
end

# Wrap angles
function nice_angle!(theta)
    while(theta < 0)
        theta += 2*pi
    end
    while(theta > 2*pi)
        theta -= 2*pi
    end
    return theta
end



# Move in direction of velocity
function move_base!(true_state, velocity, agent)
    true_state.bearings[agent] = atan2(velocity[2], velocity[1])
    ds = 0.1
    old_loc = true_state.locs[:,agent]
    new_loc = true_state.locs[:,agent] + ds*velocity
    move_style = 0
    if((new_loc[1] >= 1 && new_loc[2] >= 1) && true_state.map[round(Int,new_loc[1]),round(Int,new_loc[2])]==0)
        true_state.locs[:,agent] = new_loc
        move_style = 1 
    elseif( new_loc[1] >= 1 && true_state.map[round(Int,new_loc[1]),round(Int,true_state.locs[2,agent])] == 0)
#        true_state.locs[1,agent] = new_loc[1]
        move_style = 2
    elseif(new_loc[2] >= 1 && true_state.map[round(Int,true_state.locs[1,agent]),round(Int,new_loc[2])] == 0)
#        true_state.locs[2,agent] = new_loc[2]
        move_style = 3
    end
    if(true_state.map[round(Int,true_state.locs[1,agent]),round(Int,true_state.locs[2,agent])]==1)
        println("**********************************************")
        println("Error: Moved to occupied cell! movestyle = $move_style")
        println("Old loc: $old_loc, New loc: $new_loc, curr_state: ", true_state.locs[:,agent])
        println("**********************************************")
    end

end

### Functions used for planning paths

# Inflate obstacles to make obstacle avoidance more robust
function inflate_obstacles(map, radius)
    new_map = deepcopy(map)
    if(radius == 0)
        return new_map
    end
    for i = 1:size(map,1)
        for j = 1:size(map,2)
            if(map[i,j]==1)
                for di = -radius:1:radius
                    for dj = -radius:1:radius
                        if(i+di > 1 && i+di < size(map,1))
                            if(j+dj > 1 && j+dj < size(map,2))
                                new_map[i+di,j+dj] = 1
                            end
                        end
                    end
                end 
            end
        end
    end
    return map
end

# Predicts the reward of visiting a particular location/bearing
function predict_map(map)
    # Extend all walls in the map to be straight
    new_map = zeros(map)
    # first we search left, then we search down
    # searching happens on the original map to avoid mixing predicted/actual data
    # In this `easy mode', only look for lines that are along the axes of the map
    min_line = 3 # Number of consecutive points to be considered a `line'
    for j = 1:size(map,2)
        is_line = false
        num_consecutive = 0
        wall_length = 0
        for i = 1:size(map,1)
            state = map[i,j]
            if(state == 1)
                new_map[i,j] = 1
            end

            if(! is_line)
                # Update consecutive tracker
                if(state == 1)
                    num_consecutive+=1
                else
                    num_consecutive = 0
                end
                # Check if we just became a line
                if(num_consecutive == min_line)
                    num_consecutive = 0
                    is_line = true
                    wall_length = 0
                    # Backpropagate the line until empty space found
                    for back_i = i:-1:1
                        if(map[back_i,j]!=0)
                            new_map[back_i,j] = 1
                            wall_length += 1
                        else
                            break
                        end
                    end
                end
            else
                # If state is 0, kill line. 
                # else set new_map=1
                if(state==0 || wall_length > 50) # Limit how far forward you propogate
                    is_line = false
                else
                    new_map[i,j] = 1
                    wall_length += 1
                end
            end
        end
    end
    for i = 1:size(map,1)
        is_line = false
        num_consecutive = 0
        wall_length = 0
        for j = 1:size(map,2)
            state = map[i,j]
            if(! is_line)
                # Update consecutive tracker
                if(state == 1)
                    num_consecutive+=1
                else
                    num_consecutive = 0
                end
                # Check if we just became a line
                if(num_consecutive == min_line)
                    num_consecutive = 0
                    is_line = true
                    # Backpropagate the line until empty space found
                    for back_j = j:-1:1
                        if(map[i,back_j]!=0)
                            new_map[i,back_j] = 1
                            wall_length += 1
                        else
                            break
                        end
                    end
                end
            else
                # If state is 0, kill line. 
                # else set new_map=1
                if(state==0 || wall_length > 50)
                    is_line = false
                else
                    new_map[i,j] = 1
                    wall_length += 1
                end
            end
        end
    end
    return new_map
end


function predict_reward(map, new_map, locs, bearings) 
    # Now we predict the value by counting how much new stuff we see by visiting the given point(s?)
    num_unknown_bits = length(find(map.==0.5)) 
    data_map = deepcopy(map)
    meas = zeros(map)
    for k = 1:size(locs,2)
        meas = get_rays(new_map, vec(locs[:,k]), bearings[k])
        data_map = map_add(data_map, meas)
    end 
    # count how many bits are new
    num_new_bits = num_unknown_bits - length(find(data_map.==0.5))
#    println("Value of visiting $locs is $num_new_bits")
    if(num_new_bits < 0)
        println("Error in reward calculation. Reported $num_new_bits new bits and $num_unknown_bits unknown bits")
    end
    return num_new_bits
end



# Ray-tracing algorithm to generate measurements. Bearing is wrt E (+X)
# currently ~10x wasteful (queries 15000 times for 3600 points)
# optimize later if slow (currently 0.04s)
# There's a bug in here - in very rare cases the scan returns every other column.
function get_rays(state, agent)
    return get_rays(state.map, state.locs[:,agent], state.bearings[agent])
end
function get_rays(map, loc, bearing)
    FOV = 1.5*pi #270 degree FOV, like Hokuyo
    range = 60 # 30 meters
    nice_angle!(bearing)
    dtheta = 0.01
    xlim = size(map, 1)
    ylim = size(map, 2)
    # this is wasteful, but whatever
    meas = 0.5*ones(map)

    # heap with active rays
    count=0;

    meas[round(Int,loc[1]),round(Int,loc[2])] = map[round(Int,loc[1]),round(Int,loc[2])]
    # try each of the initial eight directions:
    for di = -1:2:1
        for dj = -1:dtheta:1
            # Check angle:
            theta = bearing - atan2(dj,di)
            theta = nice_angle!(theta)
            if(theta <= 3*pi/4 || theta >= 5*pi/4)
                # trace the ray!
                n = 0;
                while(sqrt( (n*di)^2 + (n*dj)^2) < range)
                    n+=1
                    count+=1
                    pos = [round(Int,loc[1]+n*di), round(Int,loc[2]+n*dj)]
                    if(pos[1] > 0 && pos[1] <= xlim && pos[2] > 0 && pos[2] <= ylim)
                        meas[pos[1],pos[2]] = map[pos[1],pos[2]];
                        if(map[pos[1],pos[2]] == 1) # hit an object, so kill the ray
                            break;
                        end
                    end
                end
            end
        end
    end
    for dj = -1:2:1
        for di = -1:dtheta:1
            # Check angle:
            theta = nice_angle!(bearing - atan2(dj,di))
            if(theta <= 3*pi/4 || theta >= 5*pi/4)
                # trace the ray!
                n = 0;
                while(sqrt( (n*di)^2 + (n*dj)^2) < range)
                    count+=1
                    n+=1
                    pos = ([round(Int,loc[1]+n*di), round(Int,loc[2]+n*dj)])
                    if(pos[1] > 0 && pos[1] <= xlim && pos[2] > 0 && pos[2] <= ylim)
                        meas[pos[1],pos[2]] = map[pos[1],pos[2]];
                        if(map[pos[1],pos[2]] == 1) # hit an object, so kill the ray
                            break;
                        end
                    end
                end
            end
        end
    end
    return meas
end

# Returns frontiers from list
function find_fronts(list, belief)
    mapsize = size(belief.state.map,1)
    xlim = size(belief.state.map,1); ylim = size(belief.state.map,2)
    num_gridpts = xlim*ylim
    fronts = Int64[]
    immut_list = deepcopy(list)
    for curr_ind in immut_list
        loc = ind2loc(curr_ind,mapsize)
        i = loc[1]; j = loc[2];
        # Remove from unexplored list?
        if(belief.state.map[i,j] != 0.5)
            # Relies on unexplored_list being sorted
            FMT.remove_element!(curr_ind, belief.unexplored_list)
        end
        # Frontier? add to map? 
        if(belief.state.map[i,j]==0)
            is_front = false
            # Check neighbors
            for di = -1:1:1
                for dj = -1:1:1
                    # Inbounds?
                    if( (i+di) <= xlim && (i+di) > 0 && (j+dj) <= ylim && (j+dj) > 0) 
                        if(belief.state.map[i+di,j+dj] == 0.5)
                            is_front = true
                        end
                    end
                end
            end
            if(is_front)
                push!(fronts,curr_ind)
            end
        end
    end
    return fronts
end

# Update frontier regions
function update_belief!(belief)
    information_rate = 0;
#    println("Updating belief!")
    mapsize = size(belief.state.map,1)
    xlim = size(belief.state.map,1); ylim = size(belief.state.map,2)
    num_gridpts = xlim*ylim

    # remove frontiers that are now explored:
    ind = 0
    belief.fronts = find_fronts(belief.fronts, belief)
    new_fronts = find_fronts(belief.unexplored_list, belief)
    information_rate = length(new_fronts)
    belief.fronts = FMT.add_elements!(belief.fronts, new_fronts)

#    println("Added $information_rate bits")
    return(belief, information_rate)
end

function test_goal_selection(theta)

    # Create a map that is convenient
    
    belief_state = StateData(zeros(21,21), zeros(2,10), zeros(10))
    belief       = Belief(belief_state, zeros(10), zeros(10))
    for angle = 1:1
        map = 0.5*ones(21,21)
        # 11,11 is origin
        for i = 1:21
            isfront = false
            for j = 1:21
                # compute the y value for the given x value. If less, set to 1
                yval = sign(i-11)*sqrt((i-11)^2 + (j-11)^2)*sin(theta)
                 
                if((j-11) < yval)
                    map[i,j] = 0
                end
            end
        end

        frontiers = []
        fronts_x  = []
        fronts_y  = []
        for i = 1:21
            for j = 1:21
                isfront = false
                if(map[i,j] == 0)
                    for di = -1:1
                        if(isfront)
                            continue
                        end
                        for dj = -1:1
                            if(i+di >= 1 && j+dj >= 1 && i+di <= 21 && j+dj <= 21 && map[i+di,j+dj] == 0.5)
                                isfront = true
                                push!(frontiers, loc2ind(i,j,21))
                                loc = ind2loc(frontiers[end],21)
                                push!(fronts_x,loc[1])
                                push!(fronts_y,loc[2]) 
                                break
                            end
                        end
                    end
                end        
            end
        end 

        figure(1); clf()
        imshow(map, cmap="gray", interpolation="none")
        belief.state.map = map
        belief.fronts = frontiers
        goals = select_goals(belief)
        scatter(vec(goals[2,:])-1, vec(goals[1,:])-1, color=:cyan, marker ="o")
        scatter(vec(fronts_y)-1, vec(fronts_x)-1, color=:red, marker ="x")
    end

end

# Clusters frontiers into smaller goal regions
function select_goals(belief)
    # Cluster frontiers into a goal region
    mapsize = size(belief.state.map,1)
    num_points =round(Int64, size(belief.state.map,1)*size(belief.state.map,2))

    num_fronts = size(belief.fronts,1)
    if(num_fronts == 0)
        println("No frontiers!")
    end
    clusters = round(Int64,zeros(num_points))
    cluster_size = Int64[]
    max_size = Inf

    # Cluster neighboring frontiers together
    num_clusters = 0
    conn_rad = 5
    for frontier in belief.fronts
        frontier_pt = ind2loc(frontier,mapsize)
        # Look for a cluster to join
        for di = -conn_rad:1:conn_rad
            if(clusters[frontier] != 0)
                break
            end
            for dj = -conn_rad:1:conn_rad
                if(clusters[frontier] != 0)
                    break
                end
                if(di == 0 && dj == 0)
                else
                    neighbor_ind = loc2ind(frontier_pt[1]+di, frontier_pt[2]+dj, mapsize)
                    if(neighbor_ind > 0 && neighbor_ind <= num_points)
                        if(clusters[neighbor_ind] != 0 && cluster_size[clusters[neighbor_ind]] < max_size)
                            clusters[frontier] = clusters[neighbor_ind]
                            cluster_size[clusters[neighbor_ind]]+=1
                        end
                    end
                end
            end
        end
        # no cluster to join, form own
        if(clusters[frontier] == 0)
            num_clusters+=1
            clusters[frontier] = num_clusters
            push!(cluster_size,1)
        end 
    end


    # Remove clusters that are too small (e.g. smaller than conn_rad)
    

    goals_x = Float64[]
    goals_y = Float64[]
    figure(1); 
    num_skipped =0
    for cluster_id = 1:num_clusters
        # find points in the cluster
        cluster = find(clusters.==cluster_id)
        # Take centroid as goal point
        L = length(cluster)
        if(L < conn_rad)
            # Skip this one, since its too small
            num_skipped += 1
            continue
        end
        push!(goals_x,0)
        push!(goals_y,0)
    #    cluster_x = zeros(L)
    #    cluster_y = zeros(L)
    #    for i = 1:L
    #        frontier_pt = ind2loc(cluster[i], mapsize)
    #        cluster_x[i] = frontier_pt[1]
    #        cluster_y[i] = frontier_pt[2]
    #        goals_x[cluster_id-num_skipped] += (Float64(frontier_pt[1]))/L
    #        goals_y[cluster_id-num_skipped] += (Float64(frontier_pt[2]))/L
    #    end
        # Check whether goal is in free space
        #if(belief.state.map[round(Int,goals_x[cluster_id-num_skipped]), round(Int,goals_y[cluster_id-num_skipped])] != 0)
            # _should_ rewrite this to pick a central frontier point, but for now just go random.
            mid_front = cluster[round(Int,L/2)]
            mid_pt = ind2loc(mid_front, mapsize)
            goals_x[cluster_id-num_skipped] = mid_pt[1]
            goals_y[cluster_id-num_skipped] = mid_pt[2]
        #end
    end

    # If clusters within line of sight and nearby, join together.
    
    num_clusters = length(goals_x)
    collapse = []
    for cluster_1 = 1:num_clusters-1
        if(cluster_1 in collapse)
            continue
        end
        for cluster_2 = cluster_1+1:num_clusters
            if(cluster_2 in collapse)
                continue
            end
            search_vec = [goals_x[cluster_1]-goals_x[cluster_2], goals_y[cluster_1]-goals_y[cluster_2]]
            if( norm(search_vec) < 5*conn_rad )
               #check line of sight
                line_of_sight = true
                search_pt = [goals_x[cluster_1], goals_y[cluster_2]]
                search_vec /= norm(search_vec) 
                for i = 0:round(Int,norm(search_vec))
                    search_pt += i*search_vec
                    if(belief.state.map[round(Int,search_pt[1]), round(Int,search_pt[2])]!=0)
                        line_of_sight = false
                        break
                    end
                end
                if(line_of_sight)
                    # remove the smaller cluster
                    if(length(find(clusters.==cluster_1)) < length(find(clusters.==cluster_2)))
                        push!(collapse, cluster_1)
                    else
                        push!(collapse, cluster_2)
                    end
                end
            end
        end
    end
    # Remove clusters marked for collapsing
    collapse = sort!(unique(collapse))
    while(! isempty(collapse))
        # We remove the largest first, so the indices of the smaller ones do not change.
        ind = pop!(collapse)
        splice!(goals_x,ind)
        splice!(goals_y,ind)
    end

#    scatter(vec(goals_y), vec(goals_x), color=:cyan, marker ="x")
    goals = [vec(goals_x)'; vec(goals_y)'] 

#    println("Formed $num_clusters clusters")
 
    return goals
end

# Returns list of values for each given goal
function compute_values(P, belief, goals, agent, valid)
    # Goal point is already chosen, just need to travel there
    

    times = zeros(15)
    timeindex = 0
    timelabels = []


    tic()
    num_goals = size(goals,2)
    plans = Vector{Plan}(num_goals)            # List of plans (avoid recomputation)
    if(num_goals==0)
        println("No goals!")
        return [0,0], goals, plans
    end
    num_agents = size(belief.state.bearings,1)

    # Avoid the stupid undef issue by initializing every plan:
    for i = 1:num_goals
        plans[i] = Plan(-Inf, goals[:,i], zeros(2,0)) # Put in a dummy just in case.
    end

    # We plan over the maximum liklihood occupancy grid (e.g. propogate walls)
    occ_grid = inflate_obstacles(predict_map(belief.state.map),1)
    # If inflated into present location, fix:
    occ_grid[round(Int,belief.state.locs[1,agent]), round(Int, belief.state.locs[2,agent])] = 0
    figure(43); imshow(occ_grid)
    figure(1)

    timeindex +=1
    times[timeindex] = toq()
    push!(timelabels, "compute_values: Initialization: ")

    timeindex += 1
    time_fmt = timeindex
    push!(timelabels, "compute_values: FMT: ")
    timeindex += 1
    time_predict = timeindex
    push!(timelabels, "compute_values: Predict: ")
    
    
    # Search over goals:
    values = zeros(num_goals)   # Container for value of visiting goals
    invalid = []                # List of invalid targets (used to skip in future)
    for target in valid         
        # Be sure it's feasible
        if(belief.state.map[round(Int, goals[1,target]), round(Int, goals[2,target])] == 1)
            println("Skipping target $target, in occupied space!")
            continue
        else
            # If inflated into goal location, fix:
            occ_grid[round(Int,goals[1,target]), round(Int,goals[2,target])] = 0 
        end

        tic()
        # Compute plan:
        path, cost, touched = FMT.fmtstar(vec(belief.state.locs[:,agent]), vec(goals[:,target]), round(Int,occ_grid), P)
        times[time_fmt] += toq()
        if( cost != Inf) # Add to plans if feasible
            tic()
            # Target bearing
            target_bearings = zeros(length(path)-1)
            for k = 2:length(path)
                target_bearings[k-1] = atan2(P.points[2,path[k]]-P.points[2,path[k-1]], P.points[1,path[k]]-P.points[1,path[k-1]]);
            end
            # Add a penalty for changing bearing from current:
            delta_bearing = belief.state.bearings[agent] - target_bearings[1]
            nice_angle!(delta_bearing) # 0 - 2pi
            if(delta_bearing > pi)
                delta_bearing -= 2*pi
            end
            if(abs(delta_bearing) < 3*pi/4)
                delta_bearing = 0
            end

            # Predict reward of taking given path
#            num_bits = predict_reward(belief.state.map, occ_grid, P.points[:,path[2:end]], target_bearings)
            num_bits = 100
            values[target] = num_bits/cost - abs(delta_bearing)
            if(cost < 10) # extremely close to agent
               values[target] = Inf 
            end

            # Push plan to plans container
            plans[target] = Plan(values[target], goals[:,target], P.points[:,path])
            times[time_predict] += toq()
        else
            push!(invalid, target) # Skip if infeasible 
            plans[target] = Plan(-Inf, goals[:,target], Array{Float64,2}()) # Put in a dummy just in case.
            println("Couldn't find a path from ", vec(belief.state.locs[:,agent]), " to ", vec(goals[:,target]), ". Pausing for debug.")
            println("cost is $cost, with path $path")
            # Plot the obstacle grid
            figure(42); clf();
            imshow(occ_grid)
            scatter(P.points[2,end-1:end], P.points[1,end-1:end], marker = "o", color=:white)
        end
    end
    tic()
    # Remove invalid gaols from valid list
    while(!isempty(invalid))
        g = pop!(invalid)
        FMT.remove_element!(g, valid)
    end
    timeindex +=1
    times[timeindex] = toq()
    push!(timelabels, "compute_values: Cleanup: ")


    println("compute_values: Number of iterations: ", length(valid))
    println("compute_values: Average FMT time: ", times[time_fmt]/length(valid))
    totaltime = sum(times)
    for t = 1:timeindex
        println(timelabels[t], times[t]/totaltime)
    end

    return values, valid, plans
end


# Run the assignment process
function auction_goals(P, belief, goals)

    times = zeros(20)
    timeindex = 0
    timelabels = []

    num_agents = size(belief.state.bearings,1)
    num_jobs = size(goals,2)
    if(num_jobs == 0)
        println("No jobs! task completed")
        return zeros(num_agents)
    end

    tic()
    # try hungarian method
    values = zeros(num_agents,num_jobs)
    valid = collect(1:num_jobs)
    defaults = zeros(num_agents)
    plans = Array{Plan,2}(num_agents, num_jobs)
    # Initialize
    for agent = 1:num_agents
        for job = 1:num_jobs
            plans[agent,job] = Plan(-Inf, goals[:,job], zeros(2,0))
        end
    end
    timeindex +=1
    times[timeindex] = toq()
    push!(timelabels, "Auction: Initialization: ")

    tic()
    for agent = 1:num_agents
        L0 = length(valid)
        vals, valid, plan_vec = compute_values(P,belief, goals, agent, valid)
        for job in valid
            plans[agent,job] = plan_vec[job]
        end
        values[agent,:] = vec(vals)
        defaults[agent] = findmax(vals)[2]
    end
    timeindex +=1
    times[timeindex] = toq()
    push!(timelabels, "Auction: Compute values: ")


    # What should happen: Any Inf goals are assigned to their owners
    # Remaining tasks are divied up using Hungarian

    tic()
    assignments = zeros(num_agents)
    agent_list = collect(1:num_agents)
    job_list   = collect(1:num_jobs)
    for job = 1:num_jobs
        flag = false
        for agent = 1:num_agents
            if(values[agent,job] == Inf)
                # remove the job and assign to the agent
                assignments[agent] = job
                flag = true
                FMT.remove_element!(agent, agent_list)
                println("Forcing agent $agent to take job $job")
                break
            end
        end
        if(flag)
            FMT.remove_element!(job, job_list)
        end
    end
    timeindex +=1
    times[timeindex] = toq()
    push!(timelabels, "Auction: Pre-assignment: ")
    
    tic()
    # Now assign remaining agents to remaining jobs 
    new_assignments =  Hungarian.assign_max(values[agent_list, job_list])

    j_L = length(job_list)
    a_L = length(agent_list)
    for agent = 1:a_L
        if(new_assignments[agent] > j_L || new_assignments[agent] <= 0)
            assignments[agent_list[agent]] = defaults[agent_list[agent]]
        else
            assignments[agent_list[agent]] = job_list[new_assignments[agent]]
        end
    end
    timeindex +=1
    times[timeindex] = toq()
    push!(timelabels, "Auction: Hungarian: ")

    totaltime = sum(times)
    for t = 1:timeindex
        println(timelabels[t], times[t]/totaltime)
    end

    return assignments, plans
end


# Sweeps around the point to try and generate the boolean
function test_collision_checker(pt, map)
    xlim = size(map,1)
    ylim = size(map,2)
    occ_grid = zeros(map)

    count = 0;
    for xval = 1:xlim
        for yval = 1:ylim
            count+=1
            if(FMT.collision_check(pt, [xval,yval], map))
                occ_grid[xval,yval] = 1
            end
        end
    end
    figure(5); clf(); imshow(occ_grid', cmap="gray", interpolation="none")
    return occ_grid
end

function simple_sim(num_agents)

    num_agents = 4;

    #true_map=simple_environment2(mapsize);
#    true_map = afghan_village();
    true_map = complicated_afghan_village();
    mapsize = size(true_map,1);
    xlim = mapsize; ylim = size(true_map,2)
    
    # Initialize FMT planner
    params = FMT.FMTParam(size(true_map,1), size(true_map,2), 5000, 10,1)
    P = FMT.precompute(params)
    
    # spawn agents near each other
#    num_agents = 4
## INITIALIZE
# wrap this into a function
    locs = zeros(2,num_agents)

    locs[:,1] = [100,100];
    locs[:,2] = [200,100];
    locs[:,3] = [100,300];
    locs[:,4] = [300,200];
#    locs[:,1] = [5-2,25]
#    [locs[:,1+i] = locs[:,i]+[0,1] for i = 1:num_agents-1]
#    locs[:,num_agents] = locs[:,num_agents-1] + [0,6]
    bearings = 0*2*pi*rand(num_agents)
    belief_map = 0.5*ones(true_map)  # occupancy grid
    belief_fronts = []                      # Frontier indices
    unexplored_list = collect(1:(round(Int,xlim*ylim)))
    for agent = 1:num_agents
        belief_map[locs[1,agent],locs[2,agent]] = 0
        ind = find(unexplored_list.== loc2ind(locs[1,agent],locs[2,agent],mapsize))
        if(!isempty(ind))
            splice!(unexplored_list, ind[1])
        end
    end
# that returns these:
    true_state   = StateData(true_map, locs, bearings)
    belief_state = StateData(belief_map, locs, bearings)
    belief       = Belief(belief_state, belief_fronts, unexplored_list)

    MAX_ITERS = 900 # up to 1:30 clock time.
    bits  = zeros(MAX_ITERS)  # Tracks how many bits measured to this point
    goals = zeros(num_agents) # Contains goal locations
    assignments = zeros(num_agents) # Contains index of goal assigned to agent
    plans = Array{Plan,2}()   # Contains plans of agents
    planning_period = 5       # How often to replan

    times = zeros(20)
    timeindex = 0
    timelabels = []


    for iter = 1:MAX_ITERS
        timeindex = 1

        if(iter > 1)
            figure(1);clf();
            state = deepcopy(belief.state.map)
            imshow(state,cmap="gray", interpolation="none")
            save("map.jld", "map", state);
            PyPlot.scatter(true_state.locs[2,:]-1, true_state.locs[1,:]-1, color=:green, marker=".")
        end

        println("Iter $iter")
    # Loop: measure, compound, plan, update
    ## Measure/compound:
        tic()
        for agent=1:num_agents
            # TODO incorporate noise
            meas   = get_rays(true_state,agent)
            belief.state.map = map_add(belief.state.map, meas)
        end
        times[timeindex] += toq()
        timeindex +=1
        if(iter == 1)
            push!(timelabels, "Measurement: ")
        end
    ## We can skip compounding for now - this is where they should agree on map and locations.
    ## Update frontiers and agree on goals
         
        if(iter == 1 || mod(iter, planning_period) == 1) # Time to replan!
            tic();
            belief, bits[iter] = update_belief!(belief)
            times[timeindex] += toq()
            timeindex += 1
            if(iter == 1)
                push!(timelabels, "Update belief: ")
            end
    ## Reduce frontier regions to goal points
            tic()
            goals = select_goals(belief)
            times[timeindex] += toq()
            timeindex += 1
            if(iter == 1)
                push!(timelabels, "Select goals: ")
            end
            if(size(goals,2) == 0)
                println("No goals!")
                new_iter = iter
                for k = 1:10
                    new_iter += 1
                    savefig("frameb_$new_iter.png", dpi=720)
                end
                return iter
            end
    ## Consensus here to get common goals
        # Assign agents
         
            tic()
            assignments, plans = auction_goals(P,belief,goals)
            times[timeindex] += toq()
            timeindex += 1
            if(iter == 1)
                push!(timelabels, "auction goals: ")
            end
            println("Assigning ", goals[:,assignments])
        end

        tic()
        for agent = 1:num_agents
            velocity = [0,0]
            if(assignments[agent] <= size(goals,2) && assignments[agent] > 0)
                # Go to goal
                #println(plans)
                plan = plans[agent,assignments[agent]]
                path = plan.path
                cost = plan.value
#                println("Agent $agent going to ", plan.goal)
                if(size(path,2) > 1)
                   if(norm(round(Int,path[:,2]) - round(Int,path[:,1])) < 0.1 ) # Meaning we round to the second point already
                        println("compensating for overly close goal")
                        true_state.locs[:,agent] = path[:,2]
                        belief.locs[:,agent] = path[:,2]
                        if(size(path,2) > 2) # Skip ahead to next point
                            velocity = path[:,3] - path[:,2]
                            PyPlot.plot(vec(path[2,:]),vec(path[1,:]),color=:green,linestyle=":")
                        end
                    else # Goal is far away
                        velocity = path[:,2] - path[:,1]
                        PyPlot.plot(vec(path[2,:]),vec(path[1,:]),color=:green,linestyle=":")
                    end
                else
                    println("Warning: Assigned path was not feasible!! Agent $agent, location ", assignments[agent], " at ", goals[:,assignments[agent]])
                    println("Path given is $path")
                    println("Agent plan is $plan")
                end
            else
                println("Agent $agent not assigned a valid job")
            end

            max_vel = 3
            if(norm(velocity)*36/5 > max_vel)
                velocity = velocity.*(max_vel*36./5.0/norm(velocity))
            end
#            println("agent $agent velocity = ", norm(velocity)/36*5, "m/s")
            move_base!(true_state, velocity, agent)
            # For now, just set belief.state.locs to true value 
            belief.state.locs[:,agent]  = true_state.locs[:,agent]
            belief.state.bearings[agent] = true_state.bearings[agent]
        end
            times[timeindex] += toq()
            timeindex += 1
            if(iter == 1)
                push!(timelabels, "Planning: ")
            end


            total_time = sum(times)
            for t = 1:(timeindex - 1)
                println(timelabels[t], times[t]/total_time)
            end

    clock_time = iter/10
    title("time = $clock_time seconds")
        if(iter > 1)
            #savefig("frame_$iter.png", dpi=1080)
            savefig("frameb_$iter.png")
        end
    end 

    return MAX_ITERS
end


