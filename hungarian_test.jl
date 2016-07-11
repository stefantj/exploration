include("hungarian.jl")
using Hungarian

cost_mat = [1. 2. 3. 4. 0.;
            2. 3. 4. 1. 0.;
            3. 4. 1. 2. 0.;
            4. 1. 2. 3. 0.]
assign = Hungarian.assign_max(cost_mat);
val = 0
for i = 1:4
    val+= cost_mat[i,round(Int,assign[i])]
end
println(val)
