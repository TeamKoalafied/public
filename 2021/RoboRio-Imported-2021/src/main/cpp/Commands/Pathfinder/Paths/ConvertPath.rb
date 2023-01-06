

def ConvertPath(file_base_name)
    left_filename = file_base_name + '_left_detailed.csv'
    right_filename = file_base_name + '_right_detailed.csv'
    
    c_filename = file_base_name + '.h'
    
    File.open(c_filename, 'w') do |file|
        file.puts '//' + '=' * 80
        file.puts '// ' + c_filename
        file.puts '//' + '=' * 80
        file.puts ''
        file.puts '#include <pathfinder.h>'
        file.puts ''
        left_track_variable = WriteHalfPath(file, file_base_name, 'left')
        WriteHalfPath(file, file_base_name, 'right')
        file.puts ''
        file.puts "int g_#{ConvertNameToLowerUnderscore(file_base_name)}_length = sizeof(#{left_track_variable})/sizeof(#{left_track_variable}[0]);"
    end
end

def WriteHalfPath(file, file_base_name, side)
    csv_filename = file_base_name + '_' + side + '_detailed.csv'
    lines = IO.readlines(csv_filename)
    
    variable_name = "g_#{ConvertNameToLowerUnderscore(file_base_name)}_#{side}_trajectory"
    file.puts "Segment #{variable_name}[] = {"
    first = true
    for line in lines
        if first
            first = false
            file.puts "    // #{line.chop}"
        else
            file.puts "    { #{line.chop} },"
        end
    end
    file.puts '};'
    file.puts ''
    return variable_name
end

def ConvertNameToLowerUnderscore(name)
    return name.gsub(/([A-Z]+)([A-Z][a-z])/,'\1_\2').
                gsub(/([a-z\d])([A-Z])/,'\1_\2').
                tr('-', '_').
                gsub(/\s/, '_').
                gsub(/__+/, '_').
                downcase
end

ConvertPath("RightTurn")
ConvertPath("RightSwitch")
ConvertPath("RightUTurn")
#ConvertPath("Straight3")
#ConvertPath("Straight1")
