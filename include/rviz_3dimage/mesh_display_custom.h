/* Copyright (c) 2013-2015 Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_TEXTURED_QUADS_MESH_DISPLAY_CUSTOM_H
#define RVIZ_TEXTURED_QUADS_MESH_DISPLAY_CUSTOM_H

#include <QObject>
// kinetic compatibility http://answers.ros.org/question/233786/parse-error-at-boost_join/
#ifndef Q_MOC_RUN

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRenderQueueListener.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreWindowEventUtilities.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <rviz/display.h>
#include <rviz/frame_manager.h>
#include <rviz/image/image_display_base.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz_3dimage/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <map>
#include <memory>
#include <vector>

#endif  // Q_MOC_RUN

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class FloatProperty;
class RenderPanel;
class RosTopicProperty;
class TfFrameProperty;

class MeshDisplayCustom;
class MeshImage
{
private:
    MeshDisplayCustom *mesh_display_;
    int index_;
    bool visible_;
    bool remove_;

    boost::mutex mesh_mutex_;
    shape_msgs::Mesh last_meshes_;
    geometry_msgs::Pose mesh_poses_;
    int img_widths_, img_heights_;
    float physical_widths_, physical_heights_;
    std::vector<float> border_colors_;
    float border_sizes_;

    rviz_3dimage::Image::ConstPtr cur_image_;
    int cur_image_update_count_;
    const int max_cur_image_update_count_;
    sensor_msgs::Image::ConstPtr last_images_;

    Ogre::SceneNode* mesh_nodes_;
    Ogre::ManualObject* manual_objects_;
    Ogre::MaterialPtr mesh_materials_;
    ROSImageTexture* textures_;

    Ogre::Frustum* decal_frustums_;
    // need multiple filters (back, up, down, left, right)
    Ogre::Frustum* filter_frustums_;
    Ogre::SceneNode* projector_nodes_;

    void refreshVisible();
    void load();
    void updateCamera();
    void updateMeshProperties();
    void constructQuads(const rviz_3dimage::Image::ConstPtr& images);
    void createProjector();
    void addDecalToMaterial(const Ogre::String& matName);
    shape_msgs::Mesh constructMesh(geometry_msgs::Pose mesh_origin, float width, float height, float border_size);
    void clearStates();
    // This is called by incomingMessage().
    void processImage(const sensor_msgs::Image& msg);
public:
    MeshImage(MeshDisplayCustom *mesh_display, const rviz_3dimage::Image::ConstPtr &image);
    ~MeshImage();
    void update(float wall_dt, float ros_dt);
    void updateImage(const rviz_3dimage::Image::ConstPtr& image);
    void setVisible(bool visible);
    void remove(bool val);
};
/**
 * \class MeshDisplayCustom
 * \brief Uses a pose from topic + offset to render a bounding object with shape, size and color
 */
class MeshDisplayCustom: public rviz::Display,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
    Q_OBJECT
public:
    friend class MeshImage;
    MeshDisplayCustom();
    virtual ~MeshDisplayCustom();

    // Overrides from Display
    virtual void onInitialize();
    virtual void reset();

    virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

private Q_SLOTS:
    void updateDisplayImages();

protected:

    // overrides from Display
    virtual void onEnable();
    virtual void onDisable();

    virtual void subscribe();
    virtual void unsubscribe();

private:
    void update(float wall_dt, float ros_dt);
    std::map<int, std::shared_ptr<MeshImage>> mesh_images_;
    void updateImage(const rviz_3dimage::Image::ConstPtr& image);

    void clear();
    void onCmd(const std_msgs::String::ConstPtr& msg);

    bool visible_;
    RosTopicProperty* image_topic_property_;
    TfFrameProperty* tf_frame_property_;
    ros::Subscriber image_sub_;
    ros::Subscriber cmd_sub_;

    ros::NodeHandle nh_;

    rviz_3dimage::Image::ConstPtr cur_image_;

};

}  // namespace rviz

#endif  // RVIZ_TEXTURED_QUADS_MESH_DISPLAY_CUSTOM_H


