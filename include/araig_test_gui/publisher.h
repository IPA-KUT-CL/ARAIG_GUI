
#include "ros/ros.h"

template<class MessageType>
class Publisher
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        std::string topicName;
        MessageType msg;

    public:
        
        Publisher(const std::string& topic, uint32_t queue_size)
        {
            topicName = topic;
            std::cout<<topicName<<std::endl;
            pub = nh.advertise<MessageType>(topic, queue_size);
            MessageType msg;
        }

        void publish()
        {
            pub.publish(msg);
        }

        const std::string get_topic()
        {
            return topicName;
        }

        MessageType get_message()
        {
            return msg;
        }

        void write_message(MessageType newMsg)
        {
            msg = newMsg;
        }
};