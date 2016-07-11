# Simulator module. 
using PyPlot
using Graphs
include("../Libraries/FMT/fmt.jl")
using FMT
include("buildings.jl")
include("hungarian.jl")
using Hungarian

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

function loc2ind(i,j,mapsize)
    return round(Int,round(Int,i) + (round(Int,j)-1)*mapsize)
end 

function ind2loc(ind,mapsize)
    # start with 0-based indexing then extend to 1-based:
    return [round(Int,mod(ind-1,mapsize))+1, floor(Int,(ind-1)/mapsize)+1]
end

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
function predict_reward(map, locs, bearings)
    # Extend all walls in the map to be straight
    new_map = zeros(map)
    # first we search left, then we search down
    # searching happens on the original map to avoid mixing predicted/actual data
    # In this `easy mode', only look for lines that are along the axes of the map
    min_line = 3 # Number of consecutive points to be considered a `line'
    for j = 1:size(map,2)
        is_line = false
        num_consecutive = 0
        for i = 1:size(map,1)
            state = map[i,j]
            if(state == 1)
                new_map[i,j] == 1
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
                    # Backpropagate the line until empty space found
                    for back_i = i:-1:1
                        if(map[back_i,j]!=0)
                            new_map[back_i,j] = 1
                        else
                            break
                        end
                    end
                end
            else
                # If state is 0, kill line. 
                # else set new_map=1
                if(state==0)
                    is_line = false
                else
                    new_map[i,j] = 1
                end
            end
        end
    end
    for i = 1:size(map,1)
        is_line = false
        num_consecutive = 0
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
                        else
                            break
                        end
                    end
                end
            else
                # If state is 0, kill line. 
                # else set new_map=1
                if(state==0)
                    is_line = false
                else
                    new_map[i,j] = 1
                end
            end
        end
    end


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




# Move in direction of velocity
function move_base!(true_state, velocity, agent)
    true_state.bearings[agent] = atan2(velocity[2], velocity[1])
    ds = 0.1
    old_loc = true_state.locs[:,agent]
    new_loc = true_state.locs[:,agent] + ds*velocity
    move_style = 0
    if(true_state.map[round(Int,new_loc[1]),round(Int,new_loc[2])]==0)
        true_state.locs[:,agent] = new_loc
        move_style = 1 
    elseif(true_state.map[round(Int,new_loc[1]),round(Int,true_state.locs[2,agent])] == 0)
        true_state.locs[1,agent] = new_loc[1]
        move_style = 2
    elseif(true_state.map[round(Int,true_state.locs[1,agent]),round(Int,new_loc[2])] == 0)
        true_state.locs[2,agent] = new_loc[2]
        move_style = 3
    end
    if(true_state.map[round(Int,true_state.locs[1,agent]),round(Int,true_state.locs[2,agent])]==1)
        println("**********************************************")
        println("Error: Moved to occupied cell! movestyle = $move_style")
        println("Old loc: $old_loc, New loc: $new_loc, curr_state: ", true_state.locs[:,agent])
        println("**********************************************")
    end

end


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

function nice_angle!(theta)
    while(theta < 0)
        theta += 2*pi
    end
    while(theta > 2*pi)
        theta -= 2*pi
    end
    return theta
end



# Ray-tracing algorithm to generate measurements. Bearing is wrt E (+X)
# currently ~10x wasteful (queries 15000 times for 3600 points)
# optimize later if slow (currently 0.04s)
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
    for frontier in belief.fronts
        frontier_pt = ind2loc(frontier,mapsize)
        # Look for a cluster to join
        for di = -3:1:3
            if(clusters[frontier] != 0)
                break
            end
            for dj = -3:1:3
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

    goals = zeros(2, num_clusters)
    figure(1); 
    for cluster_id = 1:num_clusters
        # find points in the cluster
        cluster = find(clusters.==cluster_id)
        # Take centroid as goal point
        L = length(cluster)
        cluster_x = zeros(L)
        cluster_y = zeros(L)
        for i = 1:L
            frontier_pt = ind2loc(cluster[i], mapsize)
            cluster_x[i] = frontier_pt[1]
            cluster_y[i] = frontier_pt[2]
            goals[1,cluster_id] += (Float64(frontier_pt[1]))/L
            goals[2,cluster_id] += (Float64(frontier_pt[2]))/L
        end

        # Check whether goal is in free space
        if(belief.state.map[round(Int,goals[1,cluster_id]), round(Int,goals[2,cluster_id])] != 0)
            # _should_ rewrite this to pick a central frontier point, but for now just go random.
                mid_front = cluster[round(Int,L/2)]
                mid_pt = ind2loc(mid_front, mapsize)
                goals[1,cluster_id] = mid_pt[1]
                goals[2,cluster_id] = mid_pt[2]
        end
    end

    scatter(vec(goals[2,:]), vec(goals[1,:]), color=:cyan, marker ="x")

#    println("Formed $num_clusters clusters")
 
    return goals
end

# Returns list of values for each given goal
function compute_values(P, belief, goals, agent, valid)
    # Goal point is already chosen, just need to travel there
    occ_grid = inflate_obstacles(round(Int, belief.state.map+0.1), 1) # treat unexplored space as obstacles
    # If inflated into location, fix:
    occ_grid[round(Int,belief.state.locs[1,agent]), round(Int, belief.state.locs[2,agent])] = 0

    num_agents = size(belief.state.bearings,1)
    if(size(goals,2)==0)
        println("No goals!")
        return [0,0], goals
    end
    num_goals = size(goals,2)

    values = zeros(num_goals)
    ind = 0
    invalid = []
    for target in valid
        ind+=1
        # Be sure it's feasible
        if(belief.state.map[round(Int, goals[1,target]), round(Int, goals[2,target])] == 1)
            println("Skipping target $target, in occupied space!")
            continue
        end
        # If inflated into goal location, fix:
        occ_grid[round(Int,goals[1,target]), round(Int,goals[2,target])] = 0 
        path, cost, touched = FMT.fmtstar(vec(belief.state.locs[:,agent]), vec(goals[:,target]), occ_grid, P)
        if(cost == Inf)
            push!(invalid, target)
            continue    
        end
        if(cost != 0 && cost != Inf)
            # Target bearing
            target_bearings = zeros(length(path)-1)
            for k = 2:length(path)
                target_bearings[k-1] = atan2(P.points[2,path[k]]-P.points[2,path[k-1]], P.points[1,path[k]]-P.points[1,path[k-1]]);
            end
            num_bits = predict_reward(belief.state.map, P.points[:,path[2:end]], target_bearings)
            values[target] = num_bits/cost
#             values[target] = -cost
        else
            println("Couldn't find a path from ", vec(belief.state.locs[:,agent]), " to ", vec(goals[:,target]), ". Pausing for debug.")
            println("cost is $cost, with path $path")
            # Plot the obstacle grid
            figure(42); clf();
            imshow(occ_grid)
#            scatter(P.points[2,:], P.points[1,:], marker = ".", color=:blue)
            scatter(P.points[2,end-1:end], P.points[1,end-1:end], marker = "o", color=:white)

#            println("Occ_grid at goal is ", occ_grid[round(Int, goals[1,target]), round(Int, goals[2,target])])
#            println("Occ_grid at start is ", occ_grid[round(Int, belief.state.locs[1,agent]), round(Int, belief.state.locs[2,agent])])
            #for pt in 1:P.params.num_pts
            #    for pt2 in P.neighborhoods[pt].inds
            #        if(!FMT.collision_check(P.points[:,pt], P.points[:,pt2], occ_grid))
            #            x = [P.points[1,pt],P.points[1,pt2]]
            #            y = [P.points[2,pt],P.points[2,pt2]]
            #            PyPlot.plot(y,x,color=:black)
            #        end
            #    end
            #end 
            # Pause until user input.
            #readline();
            figure(1)
        end
    end
    # Remove invalid gaols from valid list
    while(!isempty(invalid))
        g = pop!(invalid)
        FMT.remove_element!(g, valid)
    end

    return values, valid
end


function auction_goals(P, belief, goals)
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
    for agent = 1:num_agents
        vals, valid = compute_values(P,belief, goals, agent, valid)
        values[agent,:] = vec(vals)
        defaults[agent] = findmax(vals)[2]
    end


    t = toq()
#    println("Computing values took an average of ",t/num_agents," seconds")

    assignments =  Hungarian.assign_max(values)
#    println("Assignments are $assignments")
#    println("Value matrix is \n", round(Int, values))
    # Assign default behavior
    # Now if the job assignment is greater than the number of goals, that should be handled as a special job
    for agent = 1:num_agents
        if(assignments[agent] > num_jobs || assignments[agent] <= 0)
#            println("Agent $agent going to default target")
            assignments[agent] = defaults[agent] 
        end
    end
    return assignments
end


function simple_sim(num_agents)
    #true_map=simple_environment2(mapsize);
    true_map = afghan_village();
    mapsize = size(true_map,1);
    xlim = mapsize; ylim = size(true_map,2)
    
    # Initialize FMT planner
    params = FMT.FMTParam(size(true_map,1), size(true_map,2), 3000,10,1)
    P = FMT.precompute(params)
    
    # spawn agents near each other
#    num_agents = 4
## INITIALIZE
# wrap this into a function
    locs = zeros(2,num_agents)
    locs[:,1] = [5-2,25]
    [locs[:,1+i] = locs[:,i]+[0,1] for i = 1:num_agents-1]
    locs[:,num_agents] = locs[:,num_agents-1] + [0,6]
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

    MAX_ITERS = 600
    bits = zeros(MAX_ITERS)
    goals= zeros(num_agents)
    t0; t1; t2; t3;

    figure(1); clf();
    PyPlot.scatter(true_state.locs[2,:]-1, true_state.locs[1,:]-1,color=:green, marker=".")
    imshow(true_state.map,cmap="gray",interpolation="none")
    xlabel("X Distance (meters)")
    ylabel("Y Distance (meters)")
    title("Initial configuration with $num_agents quads")
    return 0 

    for iter = 1:MAX_ITERS

        if(iter > 1)
            figure(1);clf();
            state = deepcopy(belief.state.map)
            imshow(state,cmap="gray", interpolation="none")
            PyPlot.scatter(true_state.locs[2,:]-1, true_state.locs[1,:]-1, color=:green, marker=".")
        end

        println("Iter $iter")
    # Loop: measure, compound, plan, update
    ## Measure/compound:
        tic()
#        println("Getting rays")
        for agent=1:num_agents
            # TODO incorporate noise
            meas   = get_rays(true_state,agent)
            belief.state.map = map_add(belief.state.map, meas)
        end
#        println("Updating belief")
        t0 = toq()
        tic()
    ## We can skip compounding for now - this is where they should agree on map and locations.
    ## Update frontiers and agree on goals
        belief, bits[iter] = update_belief!(belief)
        t1=toq();
        tic()
    ## Reduce frontier regions to goal points
#        println("Selecting goals")
        goals = select_goals(belief)
        if(size(goals,2) == 0)
            println("No goals!")
# stupid way of making the last frame last a little bit longer.
            new_iter = iter
            for k = 1:10
                new_iter += 1
                savefig("frame_$new_iter.png", dpi=720)
            end
            return iter
        end
        t2 = toq();
    ## Consensus here to get common goals
#        println("Planning")
        # Assign agents
        
        assignments = round(Int, auction_goals(P,belief,goals))
        tic()
        occ_grid = inflate_obstacles(round(Int, belief.state.map+0.1),0)
        for agent = 1:num_agents
            velocity = [0,0]
            if(assignments[agent] <= size(goals,2) && assignments[agent] > 0)
                # Go to goal
                path,cost,touched = FMT.fmtstar(belief.state.locs[:,agent],goals[:,assignments[agent]], inflate_obstacles(round(Int, belief.state.map+0.1),0),P)

                velocity = P.points[:,path[2]] - P.points[:,path[1]]
                PyPlot.plot(vec(P.points[2,path]),vec(P.points[1,path]),color=:green,linestyle=":")
            else
                println("Agent $agent not assigned a valid job")
            end

            if(norm(velocity)*36/5 > 2)
                velocity = velocity.*(2*36./5.0/norm(velocity))
            end
#            println("agent $agent velocity = ", norm(velocity)/36*5, "m/s")
            move_base!(true_state, velocity, agent)
            # For now, just set belief.state.locs to true value 
            belief.state.locs[:,agent]  = true_state.locs[:,agent]
            belief.state.bearings[agent] = true_state.bearings[agent]
        end
        t3 = toq();
        t_tot = t0+t1+t2+t3;
#        println("Measurement: $t0 (", t0/t_tot)
#        println("Belief:      $t1 (", t1/t_tot)
#        println("Goals:       $t2 (", t2/t_tot)
#        println("Plan:        $t3 (", t3/t_tot)

#    if(iter == MAX_ITERS)
#    println("Drawing graph")
#    figure(3); 
#    imshow(true_map,cmap="gray", interpolation="none")
#    for pt = 1:params.num_pts
#        for pt2 in P.neighborhoods[pt].inds
#            if(pt2 > pt && !FMT.collision_check(P.points[:,pt], P.points[:,pt2], round(Int64,true_map)))
#                x = [P.points[1,pt], P.points[1,pt2]]
#                y = [P.points[2,pt], P.points[2,pt2]]
#                PyPlot.plot(y,x, color=:red)
#            end
#        end
#    end 
#    figure(1); 
#    end
    title("$iter")
        if(iter > 1)
            #savefig("frame_$iter.png", dpi=1080)
            savefig("frame_$iter.png")
        end
    end 

    return MAX_ITERS
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

