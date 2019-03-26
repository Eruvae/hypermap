#ifndef TF2_POLYGON_MSGS_H
#define TF2_POLYGON_MSGS_H


#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

namespace tf2
{

/***********/
/** Point32 **/
/***********/

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Point32& toMsg(const tf2::Vector3& in, geometry_msgs::Point32& out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Point32& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point3 message.
 * \param t_out The transformed point, as a Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::Point32& t_in, geometry_msgs::Point32& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Transform t;
    fromMsg(transform.transform, t);
    tf2::Vector3 v_in;
    fromMsg(t_in, v_in);
    tf2::Vector3 v_out = t * v_in;
    toMsg(v_out, t_out);
  }

/********************/
/** PolygonStamped    **/
/********************/

/** \brief Extract a timestamp from the header of a PolygonStamped message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t PolygonStamped message to extract the timestamp from.
 * \return The timestamp of the message. The lifetime of the returned reference
 * is bound to the lifetime of the argument.
 */
template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::PolygonStamped& p) {return p.header.stamp;}

/** \brief Extract a frame ID from the header of a PolygonStamped message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t PolygonStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message. The lifetime of the
 * returned reference is bound to the lifetime of the argument.
 */
template <>
inline
const std::string& getFrameId(const geometry_msgs::PolygonStamped &p) {return p.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const geometry_msgs::PolygonStamped &p_in, geometry_msgs::PolygonStamped &p_out, const geometry_msgs::TransformStamped& transform)
{
  p_out = p_in;
  for(geometry_msgs::Point32 &p : p_out.polygon.points)
  {
      doTransform(p, p, transform);
  }
}

// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const geometry_msgs::Polygon &p_in, geometry_msgs::Polygon &p_out, const geometry_msgs::TransformStamped& transform)
{
  p_out = p_in;
  for(geometry_msgs::Point32 &p : p_out.points)
  {
      doTransform(p, p, transform);
  }
}

inline
geometry_msgs::PolygonStamped toMsg(const geometry_msgs::PolygonStamped &in)
{
  return in;
}

inline
void fromMsg(const geometry_msgs::PolygonStamped &msg, geometry_msgs::PolygonStamped &out)
{
  out = msg;
}

inline
geometry_msgs::Polygon toMsg(const geometry_msgs::Polygon &in)
{
  return in;
}

inline
void fromMsg(const geometry_msgs::Polygon &msg, geometry_msgs::Polygon &out)
{
  out = msg;
}

}

#endif // TF2_POLYGON_MSGS_H
