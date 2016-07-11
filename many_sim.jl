include("simulator.jl")


clear_time = []
team_size = []
for num_agents = 14:-2:4 
    println("************* $num_agents Agents *********")
    push!(clear_time, simple_sim(num_agents))
    push!(team_size, num_agents)
    # Now call ffmpeg to save the video?
    run(`ffmpeg -framerate 10 -start_number 1 -i frame_%d.png -c:v libx264 test2_$num_agents.mp4`)
end

figure(142)
PyPlot.plot(team_size, clear_time/10., marker = ".")
title("Clearing time in seconds versus team size")
xlabel("Team size")
ylabel("Clearing time")
