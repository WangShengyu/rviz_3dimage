/* Copyright (c) 2013-2015 Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 * MeshDisplayCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on the rviz image display class.
 *
 * Latest changes (12/11/2012):
 * - fixed segfault issues
 */
/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 * LIABLE FOR ANY DImesh, INDImesh, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreFrustum.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_common.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/validate_floats.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz_3dimage/mesh_display_custom.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/algorithm/string.hpp>

#include <string>
#include <vector>

namespace rviz
{

MeshImage::MeshImage(MeshDisplayCustom *mesh_display, const rviz_3dimage::Image::ConstPtr &msg, bool visible)
    : index_(msg->index)
    , group_name_(msg->group_name)
    , mesh_display_(mesh_display)
    , mesh_nodes_(NULL)
    , textures_(NULL)
    , projector_nodes_(NULL)
    , manual_objects_(NULL)
    , decal_frustums_(NULL)
    , filter_frustums_(NULL)
    , max_cur_image_update_count_(5)
    , remove_(false)
{
    setVisible(visible);
    updateImage(msg);   
}

MeshImage::~MeshImage()
{
    // TODO(lucasw) switch to smart pointers
    delete mesh_nodes_;
    delete textures_;
    delete manual_objects_;
    delete decal_frustums_;
    delete filter_frustums_;
}

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
    bool valid = true;
    valid = valid && validateFloats(msg.D);
    valid = valid && validateFloats(msg.K);
    valid = valid && validateFloats(msg.R);
    valid = valid && validateFloats(msg.P);
    return valid;
}

void MeshImage::createProjector()
{
    decal_frustums_ = new Ogre::Frustum();

    projector_nodes_ = mesh_display_->scene_manager_->getRootSceneNode()->createChildSceneNode();
    projector_nodes_->attachObject(decal_frustums_);

    Ogre::SceneNode* filter_node;

    // back filter
    // delete filter_frustums_[i];
    filter_frustums_ = new Ogre::Frustum();
    filter_frustums_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
    filter_node = projector_nodes_->createChildSceneNode();
    filter_node->attachObject(filter_frustums_);
    filter_node->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y));
}

void MeshImage::addDecalToMaterial(const Ogre::String& matName)
{
    Ogre::MaterialPtr mat = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().getByName(matName);
    mat->setCullingMode(Ogre::CULL_NONE);
    Ogre::Pass* pass = mat->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);
    // pass->setLightingEnabled(true);

    // need the decal_filter to avoid back projection
    Ogre::String resource_group_name = "decal_textures_folder";
    Ogre::ResourceGroupManager& resource_manager = Ogre::ResourceGroupManager::getSingleton();
    if (!resource_manager.resourceGroupExists(resource_group_name))
    {
        resource_manager.createResourceGroup(resource_group_name);
        resource_manager.addResourceLocation(ros::package::getPath("rviz_3dimage") +
                "/tests/textures/", "FileSystem", resource_group_name, false);
        resource_manager.initialiseResourceGroup(resource_group_name);
    }
    // loads files into our resource manager
    resource_manager.loadResourceGroup(resource_group_name);

    Ogre::TextureUnitState* tex_state = pass->createTextureUnitState();  // "Decal.png");
    tex_state->setTextureName(textures_->getTexture()->getName());
    tex_state->setProjectiveTexturing(true, decal_frustums_);

    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);
    tex_state->setColourOperation(Ogre::LBO_REPLACE);  // don't accept additional effects

    tex_state = pass->createTextureUnitState("Decal_filter.png");
    tex_state->setProjectiveTexturing(true, filter_frustums_);
    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::TFO_NONE);
}

shape_msgs::Mesh MeshImage::constructMesh(geometry_msgs::Pose mesh_origin,
        float width, float height, float border_size)
{
    shape_msgs::Mesh mesh;

    Eigen::Affine3d trans_mat;
    tf::poseMsgToEigen(mesh_origin, trans_mat);

    // Rviz Coordinate System: x-right, y-forward, z-down
    // create mesh vertices and tranform them to the specified pose

    Eigen::Vector4d top_left(-width / 2.0f - border_size, 0.0f, -height / 2.0f - border_size, 1.0f);
    Eigen::Vector4d top_right(width / 2.0f + border_size, 0.0f, -height / 2.0f - border_size, 1.0f);
    Eigen::Vector4d bottom_left(-width / 2.0f - border_size, 0.0f, height / 2.0f + border_size, 1.0f);
    Eigen::Vector4d bottom_right(width / 2.0f + border_size, 0.0f, height / 2.0f + border_size, 1.0f);

    Eigen::Vector4d trans_top_left = trans_mat.matrix() * top_left;
    Eigen::Vector4d trans_top_right = trans_mat.matrix() * top_right;
    Eigen::Vector4d trans_bottom_left = trans_mat.matrix() * bottom_left;
    Eigen::Vector4d trans_bottom_right = trans_mat.matrix() * bottom_right;

    std::vector<geometry_msgs::Point> vertices(4);
    vertices.at(0).x = trans_top_left[0];
    vertices.at(0).y = trans_top_left[1];
    vertices.at(0).z = trans_top_left[2];
    vertices.at(1).x = trans_top_right[0];
    vertices.at(1).y = trans_top_right[1];
    vertices.at(1).z = trans_top_right[2];
    vertices.at(2).x = trans_bottom_left[0];
    vertices.at(2).y = trans_bottom_left[1];
    vertices.at(2).z = trans_bottom_left[2];
    vertices.at(3).x = trans_bottom_right[0];
    vertices.at(3).y = trans_bottom_right[1];
    vertices.at(3).z = trans_bottom_right[2];
    mesh.vertices = vertices;

    std::vector<shape_msgs::MeshTriangle> triangles(2);
    triangles.at(0).vertex_indices[0] = 0;
    triangles.at(0).vertex_indices[1] = 1;
    triangles.at(0).vertex_indices[2] = 2;
    triangles.at(1).vertex_indices[0] = 1;
    triangles.at(1).vertex_indices[1] = 2;
    triangles.at(1).vertex_indices[2] = 3;
    mesh.triangles = triangles;

    return mesh;
}

void MeshImage::clearStates()
{
    if (manual_objects_ != NULL)
        manual_objects_->clear();

    border_colors_.resize(4);
    for (size_t i = 0; i < 4; ++i)
    {
        border_colors_[i] = 1.0;
    }
}

void MeshImage::constructQuads(const rviz_3dimage::Image::ConstPtr& image_msg)
{
    const sensor_msgs::Image *image = &(image_msg->image);
    clearStates();

    processImage(*image);

    geometry_msgs::Pose mesh_origin;

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    float width = image_msg->mesh_width;
    float height = image_msg->mesh_height;

    mesh_origin.position.x = image_msg->pose.position.x;
    mesh_origin.position.y = image_msg->pose.position.y;
    mesh_origin.position.z = image_msg->pose.position.z;

    const geometry_msgs::Quaternion &rot = image_msg->pose.orientation;
    Eigen::Quaterniond trans_rot(rot.w, rot.x, rot.y, rot.z);
    // trans_rot = Eigen::Quaterniond(0.70710678, 0.0f, 0.0f, -0.70710678f)
                // * Eigen::Quaterniond(0.70710678, 0.70710678f, 0.0f, 0.0f);
                            

    Eigen::Quaterniond xz_quat(trans_rot);
    mesh_origin.orientation.x = xz_quat.x();
    mesh_origin.orientation.y = xz_quat.y();
    mesh_origin.orientation.z = xz_quat.z();
    mesh_origin.orientation.w = xz_quat.w();


    // set properties
    mesh_poses_ = mesh_origin;
    img_widths_ = image->width;
    img_heights_ = image->height;

    // default border size (no border)
    border_sizes_ = 0.0f;

    shape_msgs::Mesh mesh = constructMesh(mesh_origin, width, height, border_sizes_);

    physical_widths_ = width;
    physical_heights_ = height;

    boost::mutex::scoped_lock lock(mesh_mutex_);

    // create our scenenode and material
    load();

    if (!manual_objects_)
    {
        static uint32_t count = 0;
        std::stringstream ss;
        ss << "MeshObject" << count++ << "Index" << index_;
        manual_objects_ = mesh_display_->context_->getSceneManager()->createManualObject(ss.str());
        refreshVisible();
        mesh_nodes_->attachObject(manual_objects_);
    }

    // If we have the same number of tris as previously, just update the object
    if (last_meshes_.vertices.size() > 0 && mesh.vertices.size() * 2 == last_meshes_.vertices.size())
    {
        manual_objects_->beginUpdate(0);
    }
    else    // Otherwise clear it and begin anew
    {
        manual_objects_->clear();
        manual_objects_->estimateVertexCount(mesh.vertices.size() * 2);
        manual_objects_->begin(mesh_materials_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    const std::vector<geometry_msgs::Point>& points = mesh.vertices;
    for (size_t i = 0; i < mesh.triangles.size(); i++)
    {
        // make sure we have front-face/back-face triangles
        for (int side = 0; side < 2; side++)
        {
            std::vector<Ogre::Vector3> corners(3);
            for (size_t c = 0; c < 3; c++)
            {
                size_t corner = side ? 2 - c : c;  // order of corners if side == 1
                corners[corner] = Ogre::Vector3(points[mesh.triangles[i].vertex_indices[corner]].x,
                        points[mesh.triangles[i].vertex_indices[corner]].y,
                        points[mesh.triangles[i].vertex_indices[corner]].z);
            }
            Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(corners[2] - corners[0]);
            normal.normalise();

            for (size_t c = 0; c < 3; c++)
            {
                manual_objects_->position(corners[c]);
                manual_objects_->normal(normal);
            }
        }
    }

    manual_objects_->end();
    mesh_materials_->setCullingMode(Ogre::CULL_NONE);
    last_meshes_ = mesh;
}

void MeshImage::updateMeshProperties()
{
    // update color/alpha
    Ogre::Technique* technique = mesh_materials_->getTechnique(0);
    Ogre::Pass* pass = technique->getPass(0);

    Ogre::ColourValue self_illumination_color(0.0f, 0.0f, 0.0f, 0.0f);
    pass->setSelfIllumination(self_illumination_color);

    Ogre::ColourValue diffuse_color(0.0f, 0.0f, 0.0f, 1.0f);
    pass->setDiffuse(diffuse_color);

    Ogre::ColourValue ambient_color(border_colors_[0],
            border_colors_[1], border_colors_[2], border_colors_[3]);
    pass->setAmbient(ambient_color);

    Ogre::ColourValue specular_color(0.0f, 0.0f, 0.0f, 1.0f);
    pass->setSpecular(specular_color);

    Ogre::Real shininess = 64.0f;
    pass->setShininess(shininess);

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);

    mesh_display_->context_->queueRender();
}

void MeshImage::load()
{
    std::stringstream ss;
    ss << "MeshNode_" << group_name_ << "_" << index_;
    Ogre::MaterialManager& material_manager = Ogre::MaterialManager::getSingleton();
    Ogre::String resource_group_name = ss.str();

    Ogre::ResourceGroupManager& rg_mgr = Ogre::ResourceGroupManager::getSingleton();

    Ogre::String material_name = resource_group_name + "_MeshMaterial";

    if (!rg_mgr.resourceGroupExists(resource_group_name))
    {
        rg_mgr.createResourceGroup(resource_group_name);

        mesh_materials_ = material_manager.create(material_name, resource_group_name);
        Ogre::Technique* technique = mesh_materials_->getTechnique(0);
        Ogre::Pass* pass = technique->getPass(0);

        Ogre::ColourValue self_illumnation_color(0.0f, 0.0f, 0.0f, border_colors_[3]);
        pass->setSelfIllumination(self_illumnation_color);

        Ogre::ColourValue diffuse_color(border_colors_[0],
                border_colors_[1], border_colors_[2], border_colors_[3]);
        pass->setDiffuse(diffuse_color);

        Ogre::ColourValue ambient_color(border_colors_[0],
                border_colors_[1], border_colors_[2], border_colors_[3]);
        pass->setAmbient(ambient_color);

        Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
        pass->setSpecular(specular_color);

        Ogre::Real shininess = 64.0f;
        pass->setShininess(shininess);

        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        mesh_materials_->setCullingMode(Ogre::CULL_NONE);
    }

    if (mesh_nodes_ == NULL)
        mesh_nodes_ = mesh_display_->scene_node_->createChildSceneNode();
}

void MeshImage::update(float wall_dt, float ros_dt)
{
    if (cur_image_ != NULL)
    {
        // need to run these every frame in case tf has changed,
        // but could detect that.
        try
        {
            constructQuads(cur_image_);
        }
        catch (std::string& e)
        {
            mesh_display_->setStatus(StatusProperty::Error, "Display Image", e.c_str());
            return;
        }
        catch (cv_bridge::Exception& e)
        {
            mesh_display_->setStatus(StatusProperty::Error, "Display Image", e.what());
            return;
        }
        updateMeshProperties();
        // TODO(lucasw) do what is necessary for new image, but separate
        // other stuff.
    }
    else
    {
        return;
    }

    if (textures_ && !mesh_display_->image_topic_property_->getTopic().isEmpty())
    {
        try
        {
            updateCamera();
        }
        catch (UnsupportedImageEncoding& e)
        {
            mesh_display_->setStatus(StatusProperty::Error, "Display Image", e.what());
            return;
        }
        catch (std::string& e)
        {
            mesh_display_->setStatus(StatusProperty::Error, "Display Image", e.c_str());
            return;
        }
        cur_image_update_count_--;
        if (cur_image_update_count_ <= 0) {
            cur_image_ = NULL;
        }
    }
    mesh_display_->setStatus(StatusProperty::Ok, "Display Image", "ok");
}

void MeshImage::updateCamera()
{
    if (textures_->update())
    {
        last_images_ = textures_->getImage();
    }

    if (!img_heights_ || !img_widths_ ||
            !physical_widths_ || !physical_heights_ ||
            !last_images_)
    {
        throw("sizes or image not ready");
        // return false;
    }

    boost::mutex::scoped_lock lock(mesh_mutex_);

    float img_width  = img_widths_;
    float img_height = img_heights_;

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    mesh_display_->context_->getFrameManager()->getTransform(last_images_->header.frame_id,
            last_images_->header.stamp, position, orientation);

    Eigen::Affine3d trans_mat;
    tf::poseMsgToEigen(mesh_poses_, trans_mat);

    // Rotate by 90 deg to get xz plane
    trans_mat = trans_mat * Eigen::Quaterniond(0.70710678, -0.70710678f, 0.0f, 0.0f);

    float z_offset = (img_width > img_height) ? img_width : img_height;
    float scale_factor = 1.0f /
            ((physical_widths_ > physical_heights_) ?
             physical_widths_ : physical_heights_);

    Eigen::Vector4d projector_origin(0.0f, 0.0f, 1.0f / (z_offset * scale_factor), 1.0f);
    Eigen::Vector4d projector_point = trans_mat.matrix() * projector_origin;

    position = Ogre::Vector3(projector_point[0], projector_point[1], projector_point[2]);
    orientation = Ogre::Quaternion(mesh_poses_.orientation.w,
            mesh_poses_.orientation.x, mesh_poses_.orientation.y,
            mesh_poses_.orientation.z);

    // Update orientation with 90 deg offset (xy to xz)
    orientation = orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);

    // convert vision (Z-forward) frame to ogre frame (Z-out)
    orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_Z);


    // std::cout << "CameraInfo dimensions: " << last_info_->width << " x " << last_info_->height << std::endl;
    // std::cout << "Texture dimensions: " << last_image_->width << " x " << last_image_->height << std::endl;
    // std::cout << "Original image dimensions: "
    //      << last_image_->width*full_image_binning_ << " x "
    //      << last_image_->height*full_image_binning_ << std::endl;

    // If the image width/height is 0 due to a malformed caminfo, try to grab the width from the image.
    if (img_width <= 0)
    {
        ROS_ERROR("Malformed CameraInfo on camera [%s], width = 0", qPrintable(mesh_display_->getName()));
        // use texture size, but have to remove border from the perspective calculations
        img_width = textures_->getWidth() - 2;
    }

    if (img_height <= 0)
    {
        ROS_ERROR("Malformed CameraInfo on camera [%s], height = 0", qPrintable(mesh_display_->getName()));
        // use texture size, but have to remove border from the perspective calculations
        img_height = textures_->getHeight() - 2;
    }

    // if even the texture has 0 size, return
    if (img_height <= 0.0 || img_width <= 0.0)
    {
        std::string text = "Could not determine width/height of image due to malformed ";
        text += "CameraInfo (either width or height is 0) and texture.";
        throw text;
    }

    // projection matrix
    float P[12] =
    {
        1.0, 0.0, img_width / 2.0f, 0.0,
        0.0, 1.0, img_height / 2.0f, 0.0,
        0.0, 0.0, 1.0, 0.0
    };

    // calculate projection matrix
    double fx = P[0];
    double fy = P[5];

    // Add the camera's translation relative to the left camera (from P[3]);
    double tx = -1 * (P[3] / fx);
    Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
    position = position + (right * tx);

    double ty = -1 * (P[7] / fy);
    Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
    position = position + (down * ty);

    if (!validateFloats(position))
    {
        throw "CameraInfo/P resulted in an invalid position calculation (nans or infs)";
    }

    if (projector_nodes_ != NULL)
    {
        projector_nodes_->setPosition(position);
        projector_nodes_->setOrientation(orientation);
    }

    // calculate the projection matrix
    double cx = P[2];
    double cy = P[6];

    double far_plane = 100;
    double near_plane = 0.01;

    Ogre::Matrix4 proj_matrix;
    proj_matrix = Ogre::Matrix4::ZERO;

    proj_matrix[0][0] = 2.0 * fx / img_width;
    proj_matrix[1][1] = 2.0 * fy / img_height;

    proj_matrix[0][2] = 2.0 * (0.5 - cx / img_width);
    proj_matrix[1][2] = 2.0 * (cy / img_height - 0.5);

    proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
    proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);

    proj_matrix[3][2] = -1;

    if (decal_frustums_ != NULL)
        decal_frustums_->setCustomProjectionMatrix(true, proj_matrix);

    mesh_display_->setStatus(StatusProperty::Ok, "Time", "ok");
    mesh_display_->setStatus(StatusProperty::Ok, "Camera Info", "ok");

    if (filter_frustums_ == NULL) {
        createProjector();
    }
    if (mesh_nodes_ != NULL && !mesh_materials_.isNull())
    {
        addDecalToMaterial(mesh_materials_->getName());
    }
}

void MeshImage::processImage(const sensor_msgs::Image& msg)
{
    // std::cout<<"camera image received"<<std::endl;
    cv_bridge::CvImagePtr cv_ptr;

    // simply converting every image to RGBA
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);

    // update image alpha
    /*
    for(int i = 0; i < cv_ptr->image.rows; i++)
    {
        for(int j = 0; j < cv_ptr->image.cols; j++)
        {
            cv::Vec4b& pixel = cv_ptr->image.at<cv::Vec4b>(i,j);
            pixel[0] = (i + j) % 256;
            pixel[1] = 0;
            pixel[2] = 0;
            pixel[3] = 255;
        }
    }
    */

    // add completely white transparent border to the image so that it won't replicate colored pixels all over the mesh
    cv::Scalar value(255, 255, 255, 0);
    cv::copyMakeBorder(cv_ptr->image, cv_ptr->image, 1, 1, 1, 1, cv::BORDER_CONSTANT, value);
    cv::flip(cv_ptr->image, cv_ptr->image, -1);

    // Output modified video stream
    if (textures_ == NULL)
        textures_ = new ROSImageTexture();

    textures_->addMessage(cv_ptr->toImageMsg());
}

void MeshImage::updateImage(const rviz_3dimage::Image::ConstPtr& image)
{
    cur_image_ = image;
    cur_image_update_count_ = max_cur_image_update_count_;
    resume();
}

void MeshImage::resume()
{
    remove_ = false;
    refreshVisible();
}

void MeshImage::remove()
{
    remove_ = true;
    refreshVisible();
}

void MeshImage::refreshVisible()
{
    if (manual_objects_ != NULL)
    {
        manual_objects_->setVisible(visible_ && !remove_);
    }
}

void MeshImage::setVisible(bool visible)
{
    visible_ = visible;
    refreshVisible();
}

MeshDisplayCustom::MeshDisplayCustom()
    : Display()
    , default_visible_(false)
{
    image_topic_property_ = new RosTopicProperty("Image Topic", "",
            QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
            "Image topic to subscribe to.",
            this, SLOT(updateDisplayImages()));
    // TODO(lucasw) add controls to switch which plane to align to?
    tf_frame_property_ = new TfFrameProperty("Quad Frame", "my_frame",
            "Align the image quad to the xy plane of this tf frame",
            this, 0, true);
}

MeshDisplayCustom::~MeshDisplayCustom()
{
    unsubscribe();

    // TODO(lucasw) clean up other things
    delete image_topic_property_;
    delete tf_frame_property_;
}

void MeshDisplayCustom::update(float wall_dt, float ros_dt)
{
    for (auto group_it = groups_.begin(); group_it != groups_.end(); group_it++)
    {
        Group &group = group_it->second;
        for (auto it = group.mesh_images.begin(); it != group.mesh_images.end(); it++)
        {
            it->second->update(wall_dt, ros_dt);
        }
    }
}

void MeshDisplayCustom::onInitialize()
{
    tf_frame_property_->setFrameManager(context_->getFrameManager());
    Display::onInitialize();
}

void MeshDisplayCustom::updateImage(const rviz_3dimage::Image::ConstPtr& image)
{
    if (groups_.find(image->group_name) == groups_.end())
    {
        groups_[image->group_name].visible = default_visible_;
    }
    Group &group = groups_[image->group_name];
    int index = image->index;
    auto it = group.mesh_images.find(index);
    if (it == group.mesh_images.end())
    {
        group.mesh_images[index] = std::make_shared<MeshImage>(this, image, group.visible);
    }
    else
    {
        it->second->updateImage(image);
    }
}

void MeshDisplayCustom::onCmd(const std_msgs::String::ConstPtr &msg)
{
    std::string raw_cmd = msg->data;
    std::vector<std::string> cmds;
    boost::split(cmds, raw_cmd, boost::is_any_of("|"));
    
    if (cmds[0] == "hide" || cmds[0] == "show")
    {
        auto setGroupVisible = [](Group &group, bool show)
        {
            group.visible = show;
            for (auto it = group.mesh_images.begin(); it != group.mesh_images.end(); it++)
            {
                it->second->setVisible(show);
            }
        };
        bool show = cmds[0] == "show";
        if (cmds.size() == 1)
        {
            default_visible_ = show;
            for (auto it = groups_.begin(); it != groups_.end(); it++)
            {
                setGroupVisible(it->second, show);
            }
        }
        else
        {
            std::string group_name = cmds[1];
            setGroupVisible(groups_[group_name], show);
        }
    }
    else if (cmds[0] == "clear")
    {
        auto clearGroup = [](const std::map<std::string, Group>::iterator &it)
        {
            Group &group = it->second;
            for (auto it = group.mesh_images.begin(); it != group.mesh_images.end(); it++)
            {
                it->second->remove();
            }
        };
        if (cmds.size() == 1)
        {
            for (auto it = groups_.begin(); it != groups_.end(); it++)
            {
                clearGroup(it);
            }
        }
        else
        {
            std::string group_name = cmds[1];
            auto it = groups_.find(group_name);
            if (it != groups_.end())
            {
                clearGroup(it);
            }
        }
    }
}

void MeshDisplayCustom::updateDisplayImages()
{
    unsubscribe();
    subscribe();
}

void MeshDisplayCustom::subscribe()
{
    if (!isEnabled())
    {
        return;
    }

    if (!image_topic_property_->getTopic().isEmpty())
    {
        try
        {
            image_sub_ = nh_.subscribe("image3d",
                    1000, &MeshDisplayCustom::updateImage, this);
            setStatus(StatusProperty::Ok, "Display Images Topic", "ok");
            
            cmd_sub_ = nh_.subscribe("image3d_cmd",
              1000, &MeshDisplayCustom::onCmd, this);
        }
        catch (ros::Exception& e)
        {
            setStatus(StatusProperty::Error, "Display Images Topic", QString("Error subscribing: ") + e.what());
        }
    }
}

void MeshDisplayCustom::unsubscribe()
{
    image_sub_.shutdown();
}


void MeshDisplayCustom::onEnable()
{
    subscribe();
}

void MeshDisplayCustom::onDisable()
{
    unsubscribe();
}

void MeshDisplayCustom::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
}

void MeshDisplayCustom::clear()
{
    context_->queueRender();

    setStatus(StatusProperty::Warn, "Image", "No Image received");
}

void MeshDisplayCustom::reset()
{
    Display::reset();
    clear();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::MeshDisplayCustom, rviz::Display)
