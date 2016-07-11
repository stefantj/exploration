# Code snippets for running hungarian algorithm
# Based on the implementation from Google:
# https://raw.githubusercontent.com/google/or-tools/master/src/algorithms/hungarian.cc
#

module Hungarian

type HungarianVars
    marks::Array{Int8,2}
    matrix_size::Int64
    costs::Array{Float64,2}
    max_cost::Float64
    rowCovered::Array{Bool,1}
    colCovered::Array{Bool,1}
    starsincol::Array{Int64,1}
    preimage::Array{Int64,1}
    image::Array{Int64,1}
    zero_col::Int64
    zero_row::Int64
    width::Int64
    height::Int64
    state
end

function assign_max(value_matrix::Array{Float64,2})
    # Here we augment and invert
    num_jobs = size(value_matrix,2)
    num_agents = size(value_matrix,1)
#    println("Assigning $num_jobs tasks to $num_agents agents")
    matrix_size = max(num_jobs, num_agents)

    costs = zeros(matrix_size,matrix_size)
    max_cost = maximum(value_matrix)
    for job = 1:num_jobs
        for agent = 1:num_agents
            costs[agent,job] = max_cost-value_matrix[agent,job]
        end
    end
    P = HungarianVars(zeros(matrix_size,matrix_size), 
                      matrix_size, 
                      costs,
                      max_cost,
                      falses(matrix_size),
                      falses(matrix_size),
                      zeros(matrix_size),
                      zeros(2*matrix_size),
                      zeros(2*matrix_size),  
                      0,
                      0,
                      num_jobs,
                      num_agents,
                      1)
    sanity_checker = 0
    t = zeros(7)
    while(P.state != 0 && sanity_checker < 1000)
        if(P.state == 1)
            sanity_checker +=1
            P = hungarian_step1(P)
        elseif(P.state == 2)
            sanity_checker +=1
            P = hungarian_step2(P)
        elseif(P.state == 3)
            sanity_checker +=1
            P = hungarian_step3(P)
        elseif(P.state == 4)
            sanity_checker +=1
            P = hungarian_step4(P)
        elseif(P.state == 5)
            sanity_checker +=1
            P = hungarian_step5(P)
        elseif(P.state == 6)
            sanity_checker +=1
            P = hungarian_step6(P)
        else
            println("Error in steps")
        end
    end
    if(sanity_checker >= 1000)
        println("Hungarian algorithm got stuck. Exiting")
        return zeros(num_agents)
    end
    # Run assignment
    assignment = get_assignments(P)
#    println("assigning ", assignment[1:num_agents])
    return round(Int,assignment[1:num_agents])
end

function get_assignments(P)
    assignments = zeros(P.matrix_size)
    for row = 1:P.matrix_size
        for col = 1:P.matrix_size
            if(P.marks[row,col] == 2)
                assignments[row] = col
            end
        end
    end
    return assignments
end


function hungarian_step1(P::HungarianVars)

    # Step 1: Remove smallest element from each row
    for row = 1:P.matrix_size
        mincost = minimum(P.costs[row,:])
        P.costs[row,:] -= mincost
    end
    # Proceed to step 2
    P.state = 2
    return P
end

function hungarian_step2(P::HungarianVars)
    # Step 2: Run lines
    P.colCovered = falses(P.matrix_size)
    P.rowCovered = falses(P.matrix_size)
    for row = 1:P.matrix_size
        if(P.rowCovered[row])
            continue
        end 
        for col = 1:P.matrix_size
            if(P.colCovered[col])
                continue
            end
            if(P.costs[row,col] == 0)
                # Mark as star
                P.marks[row,col]= 2 
                P.starsincol[col] += 1
                P.colCovered[col]=true
                P.rowCovered[row]=true
                break
            end
        end
    end
    
    # Print stars
    P.colCovered = falses(P.matrix_size)
    P.rowCovered = falses(P.matrix_size)
    P.state = 3
    return P
end

function hungarian_step3(P)
    num_covered = 0
    for col=1:P.matrix_size
        if(P.starsincol[col]>0)
            P.colCovered[col]=true
            num_covered+=1
        end
    end
    if(num_covered >= P.matrix_size)
        P.state = 0
    else
        P.state = 4
    end
    return P
end

function hungarian_step4(P)
  while(true)
    star_col = 0
    zero_row = 0
    zero_col = 0

    for row = 1:P.matrix_size
        if(P.rowCovered[row])
            continue
        end
        if(P.zero_row != 0)
            break
        end
        for col = 1:P.matrix_size
            if(P.colCovered[col])
                continue
            end
            if(P.costs[row,col]==0)
                zero_row = row
                zero_col = col
                break
            end
        end
    end
    
    if(zero_row == 0)
        P.state = 6
        return P
    end

    # Mark as prime
    P.marks[zero_row,zero_col] = 1
    for col = 1:P.matrix_size
        if(P.marks[zero_row, col] == 2)
            star_col = col
            break
        end
    end
    

    if(star_col != 0)
        P.rowCovered[zero_row] = true
        P.colCovered[star_col] = false
    else
        P.preimage[1] = zero_row
        P.image[1] = zero_col
        P.state = 5
        return P 
    end
  end
end


function hungarian_step5(P)
    done=false
    row = 0
    count = 1
    while(!done)
        # IF no stars in col, then done
        if(P.starsincol[P.image[count]] == 0)
            done = true
        else # do the rest of the things
        # Find star in the col
            for star_row = 1:P.matrix_size
                if(P.marks[star_row,P.image[count]] == 2)
                    row = star_row
                    break
                end
            end
            count += 1
            P.preimage[count] = row
            P.image[count]    = P.image[count - 1]
            col = 0
            # Find prime in row 
            for prime_col = 1:P.matrix_size
                if(P.marks[P.preimage[count], prime_col]==1)
                    col = prime_col
                    break
                end
            end
            count += 1
            P.preimage[count] = P.preimage[count-1]
            P.image[count] = col 
        end
    end

    for i = 1:count
        row = P.preimage[i]
        col = P.image[i]
        if(P.marks[row,col] == 2)
            P.marks[row,col] = 0
            P.starsincol[col] -=1
        else
            P.marks[row,col] = 2
            P.starsincol[col] += 1
        end
    end
    # Clear covers and primes
    for row = 1:P.matrix_size
        for col = 1:P.matrix_size
            if(P.marks[row,col] == 1)
                P.marks[row,col] = 0
            end
        end
    end
    P.colCovered = falses(P.matrix_size)
    P.rowCovered = falses(P.matrix_size)
    P.state = 3
    return P
end


function hungarian_step6(P)
    minval = Inf
    for row = 1:P.matrix_size
        if(P.rowCovered[row])
            continue
        end
        for col = 1:P.matrix_size
            if(P.colCovered[col])
                continue
            end
            if(P.costs[row,col] < minval)
                minval = P.costs[row,col]
            end
        end
    end

    for row = 1:P.matrix_size
        for col = 1:P.matrix_size
            if(P.rowCovered[row])
                P.costs[row,col] += minval
            end
            if(!P.colCovered[col])
                P.costs[row,col] -= minval
            end
        end
    end
    P.state = 4
    return P
end

end
