#include "move3d_ros_robot_split.hpp"
#include <libmove3d/planners/utils/misc_functions.hpp>
#include <libmove3d/planners/utils/NumsAndStrings.hpp>

using std::cout;
using std::endl;

Move3DRosRobotSplit::Move3DRosRobotSplit()
    : split_trajectories_(false)
{

}

bool Move3DRosRobotSplit::loadSplits(std::string filename)
{
    std::vector<std::string> strings = load_strings_from_file( filename );
    if( strings.empty() )
        return false;

    if( strings.size() % 4 != 0 )
        return false;

    splits_.clear();

    cout << "strings size : " << strings.size() << endl;

    for( size_t i=0; i<strings.size(); i += 4 )
    {
        cout << " i : " << i << endl;

        double t_sec, t_nano;
        convert_text_to_num<double>( t_sec,  strings[i+0], std::dec );
        convert_text_to_num<double>( t_nano, strings[i+1], std::dec );
        ros::Time t_ini( t_sec, t_nano );

        convert_text_to_num<double>( t_sec,  strings[i+2], std::dec );
        convert_text_to_num<double>( t_nano, strings[i+3], std::dec );
        ros::Time t_end( t_sec, t_nano );

        splits_.push_back( split_t(
                               t_ini.toSec(),
                               t_end.toSec()));

        cout << "split init : " << t_ini.toSec() << endl;
    }

    for( size_t i=0; i<splits_.size(); i++ ){
        cout << "splits " << i
             << " = t_ini: " << splits_[i].t_init_
             << " = t_end: " << splits_[i].t_end_
             << endl;
    }

    split_trajectories_ = true;

    trajectories_.clear();
    configs_.clear();
    current_split_id_ = 0;

    return true;
}

bool Move3DRosRobotSplit::saveToFiles()
{
    if( trajectories_.empty() )
        return false;

    for( size_t i=0; i<trajectories_.size(); i++ )
    {
        std::stringstream ss;
        ss.str("");
        ss << trajectories_folder_
           << "/robot_"
           << std::setw(3) << std::setfill( '0' ) << i
           << ".traj";

        // Resample at 100 Hz constant dt
        Move3D::Trajectory traj( HRICS::motion_to_traj( trajectories_[i] ,
                                 robot_ ) );

        // Save to file
        traj.saveToFile( ss.str() );

        ROS_INFO("SAVE TRAJECTORY %d for robot %s with %d waypoints at %s",
                 i,
                 robot_->getName().c_str(),
                 traj.getNbOfViaPoints(),
                 ss.str().c_str()
                 );

    }

    trajectories_.clear();
    return true;
}

bool Move3DRosRobotSplit::storeConfiguration( Move3D::confPtr_t q, double time )
{
    if( current_split_id_ >= splits_.size() ) {
        saveToFiles();
        return false;
    }

    // Check if after the end of split
    if( time > splits_[ current_split_id_ ].t_end_ )
    {
        trajectories_.push_back( configs_ );
        configs_.clear();

        current_split_id_++;
    }

    if( current_split_id_ >= splits_.size() )
        return false;

    // Check if after begining of split
    if( time >= splits_[ current_split_id_ ].t_init_ )
    {
        double dt = 0.0;

        if( !configs_.empty() )
        {
            dt = time - t_last_insert_;
        }
        t_last_insert_ = time;

        cout << "dt : " << dt << endl;

        configs_.push_back( std::make_pair( dt, q->copy() ) );
    }

    return true;
}
