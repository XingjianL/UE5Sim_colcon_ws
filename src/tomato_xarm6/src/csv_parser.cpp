#include "tomato_xarm6/csv_parser.hpp"

std::vector<double> tomato_xarm6::csv2vector(const std::string& csv)
{
    std::ifstream file(csv);
    if (!file.is_open()){
        RCLCPP_ERROR(rclcpp::get_logger("tomato_xarm6"), "Incorrect csv file path!");
    }
    std::vector<double> result;
    std::string item;
    // handling both ',' and '\n' delimiters in csv file
    while (std::getline(file, item, ','))
    {
        std::stringstream ss(item);
        while(std::getline(ss, item, '\n')){
            //printf("%s %f %d\n", item.c_str(), std::stod(item), result.size());
            result.push_back(std::stod(item));
        }
    }
    file.close();
    return result;
}

std::vector<tomato_xarm6::setpoints> tomato_xarm6::read_robot_setpoints(const std::string& position_csv, const std::string& orientation_csv)
{
    std::vector<double> position = csv2vector(position_csv);
    std::vector<double> orientation = csv2vector(orientation_csv);
    if (position.size() != orientation.size()){
        RCLCPP_ERROR(rclcpp::get_logger("tomato_xarm6"), "Incorrect csv file content between position and orientation!");
    }
    std::vector<tomato_xarm6::setpoints> setpoints;
    for (size_t i = 0; i < position.size(); i+=3){
        setpoints.push_back({position[i], position[i+1], position[i+2], orientation[i], orientation[i+1], orientation[i+2]});
    }
    return setpoints;

}