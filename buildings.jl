function make_room(SW_corner, NE_corner, door_locs, door_size, map)
    # draw box, then erase door
    i_min = SW_corner[1]
    i_max = NE_corner[1]
    j_min = SW_corner[2]
    j_max = NE_corner[2]

    for i = i_min:i_max
        map[i,j_min] = 1
        map[i,j_max] = 1
    end
    for j = j_min:j_max
        map[i_min,j] = 1
        map[i_max,j] = 1
    end

    start = j_min+round(Int, (j_max-j_min-door_size)/2)
    if 'E' in door_locs
        for j = start:start+door_size
            map[i_max,j] = 0
        end
    end
    if 'W' in door_locs
        for j = start:start+door_size
            map[i_min,j] = 0
        end
    end
    start = i_min+round(Int, (i_max-i_min-door_size)/2)
    if 'N' in door_locs
        for i = start:start+door_size
            map[i,j_max] = 0
        end 
    end
    if 'S' in door_locs
        for i = start:start+door_size
            map[i,j_min] = 0
        end
    end 
    return map
end


function afghan_village()
    xlim = 200
    ylim = 150
    # Add some exterior too:
    left_start = 25
    right_end  = xlim-left_start
    bottom_start = 25
    top_end      = ylim - bottom_start
    # Rough estimate based off of Brandon's drawing:
    map = zeros(xlim,ylim)
    # Draw this room-by-room (somewhat painful
    cell_size = 18
    door_width = 6
    # Room 1:
    room_width = round(Int,3.5*cell_size)
    room_height = round(Int,1.8*cell_size)
    SE_corner = [left_start,bottom_start]; NW_corner = SE_corner + [room_width,room_height] 
    map = make_room(SE_corner, NW_corner, "N", door_width, map)
    # Room 2:
    SE_corner = SE_corner + [0,room_height]
    room_width = round(Int,1.5*cell_size)
    room_height = round(Int,1.1*cell_size)
    NW_corner = SE_corner + [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "N", door_width, map)
    # Room 3:
    SE_corner = SE_corner + [0,room_height]
    room_height = round(Int,2*cell_size)
    NW_corner = SE_corner + [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "SE", door_width, map)
    # Room 4: 
    SE_corner = SE_corner + [0,room_height]
    room_height = round(Int,1.1*cell_size)
    NW_corner = SE_corner + [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "E", door_width, map)
    # Room 5:
    SE_corner = SE_corner + [room_width, 0]
    room_width = round(Int,2.7*cell_size)
    NW_corner = SE_corner + [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "WS", door_width, map)
    # Room 6:
    SE_corner = SE_corner + [room_width, 0]
    NW_corner = SE_corner + [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "ES", door_width, map)
   # Room 7:
    SE_corner = SE_corner + [room_width, 0]
    room_width = room_height
    NW_corner = SE_corner + [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "W", door_width, map)
    # Room 8:
    room_width = round(Int,1.1*cell_size)
    NW_corner = NW_corner + [room_width, 0]
    room_height = round(Int,3*cell_size)
    SE_corner = NW_corner - [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "WS", door_width, map)
    # Room 9:
    NW_corner = NW_corner - [0, room_height]
    room_height = round(Int,1.5*cell_size)
    SE_corner = NW_corner - [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "NWS", door_width, map)
    # Room 10:
    NW_corner = NW_corner - [0, room_height]
    room_height = round(Int,1.5*cell_size)
    SE_corner = NW_corner - [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "N", door_width, map)
    # Room 11:
    NW_corner = NW_corner - [room_width, 0]
    room_width = round(Int,1.9*cell_size)
    SE_corner = NW_corner - [room_width,room_height]
    map = make_room(SE_corner, NW_corner, "N", door_width,map) 

    return map
end


function test_building()
    map = afghan_village()
    figure(1); clf(); 
    imshow(map,cmap="gray")
end

function simple_environment2(map_size)
    base_map = simple_environment(map_size)
    # add random 10x10 obstructions

    for k=1:20
        upper_corner = round(Int,rand(2)*128)
        for i = upper_corner[1]:(upper_corner[1]+10)
            for j = upper_corner[2]:(upper_corner[2]+10)
                if(i >= 128 || j >= 128)
                else
                    base_map[i+1,j+1] = 1
                end
            end
        end
    end
    return base_map
end



function simple_environment(map_size)
    door_size = 6;
    map = zeros(map_size,map_size)
    #always has a border
    for i = 1:map_size
        map[i,1] = 1
        map[i,map_size]=1
        map[1,i] = 1
        map[map_size,i]=1
    end

    # add 3 rooms per side
    room_length = round(Int,map_size/(3.0))
    room_width  = round(Int,map_size/(2.5))

    for i = 1:room_width
        if( i > (room_width-door_size)/2 && i < (room_width+door_size)/2)
        else
            map[i, room_length] = 1
        end
        map[i, map_size-room_length] = 1
    end
    for i = (map_size-room_width):map_size
        if( (map_size - i) > (room_width - door_size)/2 && (map_size-i) < (room_width + door_size)/2)
        else
            map[i,map_size-room_length] = 1
        end
        map[i,room_length] = 1
    end

    for j = 1:round(Int,(room_length-door_size)/2) 
        map[room_width, j] = 1
        map[map_size-room_width,j] = 1
        map[room_width, map_size-j] = 1
        map[map_size-room_width,map_size-j] = 1
    end
    for j = 1:room_length-door_size
        map[room_width, round(Int,ceil(room_length/2+door_size+j))] = 1
        map[map_size-room_width,round(Int,ceil(room_length/2+door_size+j))] = 1
        map[room_width, round(Int,map_size - ceil((room_length/2+door_size+j)))] = 1
        map[(map_size-room_width),round(Int,map_size -ceil((room_length/2+door_size+j)))] = 1
    end
    return map
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


function complicated_afghan_village()
    map1 = afghan_village()
    map2 = map_flip(1,map1)
    map4 = map_flip(2,map2)
    map3 = map_flip(2,map1)
    map  = concat_4_maps(map3,map4,map1,map2)
    # Draw in borders
    for i = 1:size(map,1)
        map[i,1] = 1
        map[i,size(map,2)] = 1
    end  
    for j = 1:size(map,2)
        map[1,j] = 1
        map[size(map,1),j] = 1
    end  
    return map
end


function print_cpp_map(filename, map)
# maps are 3D
    f = open(filename,"w")
    write(f, "float map[1][");
    write(f, string(size(map,1)));
    write(f, "][");
    write(f, string(size(map,2)));
    write(f, "] = {{");
    for i=1:size(map,1)
        write(f,"{");
        for j = 1:size(map,2)-1
            write(f, string(map[i,j]))
            write(f, ", ")
        end
        write(f,string(map[i,size(map,2)]));
        write(f, "},");
    end
    write(f, "}};");
    close(f)
end



