#include "tf_node.h"

TF_Node* tf_node;

void sigintCB( int sig )
{
    tf_node->close();
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_node",ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    signal(SIGINT,&sigintCB);

    tf_node = new TF_Node(&nh,&nh_priv);
    while(ros::ok())
    {
        ros::spinOnce();
        tf_node->loop();
    }

    return 0;
}
