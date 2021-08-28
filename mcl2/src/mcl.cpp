#include "mcl_localization/mcl.h"
#include <math.h>

//Debug field points
#define DEBUG_MAT

MCL::MCL()
{
  use_heading = false;
  field_lines = false;
  vision_debug = false;
}

/*
 * Init()
 * Initialize the field matrix for field points detection.
 * Initialize particles, load config and pass them to ui
 */

void MCL::init(int argc, char** argv)
{
  field = cv::Mat::zeros(FIELD_HEIGHT+(2*BORDER), FIELD_WIDTH+(2*BORDER), CV_8UC1);

  //감지할 라인 정의
  cv::circle(field, cv::Point((FIELD_WIDTH/2)+BORDER, (FIELD_HEIGHT/2)+BORDER), 75, cv::Scalar(255));
  cv::line(field, cv::Point(BORDER,BORDER), cv::Point(BORDER+FIELD_WIDTH, BORDER), cv::Scalar(255));
  cv::line(field, cv::Point(BORDER,BORDER+FIELD_HEIGHT), cv::Point(BORDER+FIELD_WIDTH, BORDER+FIELD_HEIGHT), cv::Scalar(255));
  cv::line(field, cv::Point(BORDER,BORDER+50), cv::Point(BORDER+100, BORDER+50), cv::Scalar(255));
  cv::line(field, cv::Point(BORDER,BORDER+FIELD_HEIGHT-50), cv::Point(BORDER+100, BORDER+FIELD_HEIGHT-50), cv::Scalar(255));
  cv::line(field, cv::Point(FIELD_WIDTH+BORDER-100,BORDER+50), cv::Point(FIELD_WIDTH+BORDER, BORDER+50), cv::Scalar(255));
  cv::line(field, cv::Point(FIELD_WIDTH+BORDER-100,BORDER+FIELD_HEIGHT-50), cv::Point(FIELD_WIDTH+BORDER, BORDER+FIELD_HEIGHT-50), cv::Scalar(255));
  cv::line(field, cv::Point((FIELD_WIDTH/2)+BORDER, BORDER), cv::Point((FIELD_WIDTH/2)+BORDER, FIELD_HEIGHT+BORDER), cv::Scalar(255));
  cv::line(field, cv::Point(BORDER+100, BORDER+50), cv::Point(BORDER+100, BORDER+FIELD_HEIGHT-50), cv::Scalar(255));
  cv::line(field, cv::Point(FIELD_WIDTH+BORDER-100,BORDER+50), cv::Point(FIELD_WIDTH+BORDER-100, BORDER+FIELD_HEIGHT-50), cv::Scalar(255));
  cv::line(field, cv::Point(BORDER, BORDER), cv::Point(BORDER, BORDER+FIELD_HEIGHT), cv::Scalar(255));
  cv::line(field, cv::Point(BORDER+FIELD_WIDTH, BORDER), cv::Point(BORDER+FIELD_WIDTH, BORDER+FIELD_HEIGHT), cv::Scalar(255));
  //

  horizontal_field_lines.push_back(LineSegment(cv::Point(BORDER,BORDER), cv::Point(BORDER+FIELD_WIDTH, BORDER)));
  horizontal_field_lines.push_back(LineSegment(cv::Point(BORDER,BORDER+FIELD_HEIGHT), cv::Point(BORDER+FIELD_WIDTH, BORDER+FIELD_HEIGHT)));
  horizontal_field_lines.push_back(LineSegment(cv::Point(BORDER,BORDER+50), cv::Point(BORDER+100, BORDER+50)));
  horizontal_field_lines.push_back(LineSegment(cv::Point(BORDER,BORDER+FIELD_HEIGHT-50), cv::Point(BORDER+100, BORDER+FIELD_HEIGHT-50)));
  horizontal_field_lines.push_back(LineSegment(cv::Point(FIELD_WIDTH+BORDER-100,BORDER+50), cv::Point(FIELD_WIDTH+BORDER, BORDER+50)));
  horizontal_field_lines.push_back(LineSegment(cv::Point(FIELD_WIDTH+BORDER-100,BORDER+FIELD_HEIGHT-50), cv::Point(FIELD_WIDTH+BORDER, BORDER+FIELD_HEIGHT-50)));

  vertical_field_lines.push_back(LineSegment(cv::Point((FIELD_WIDTH/2)+BORDER, BORDER), cv::Point((FIELD_WIDTH/2)+BORDER, FIELD_HEIGHT+BORDER)));
  vertical_field_lines.push_back(LineSegment(cv::Point(BORDER+100, BORDER+50), cv::Point(BORDER+100, BORDER+FIELD_HEIGHT-50)));
  vertical_field_lines.push_back(LineSegment(cv::Point(FIELD_WIDTH+BORDER-100,BORDER+50), cv::Point(FIELD_WIDTH+BORDER-100, BORDER+FIELD_HEIGHT-50)));
  vertical_field_lines.push_back(LineSegment(cv::Point(BORDER, BORDER), cv::Point(BORDER, BORDER+FIELD_HEIGHT)));
  vertical_field_lines.push_back(LineSegment(cv::Point(BORDER+FIELD_WIDTH, BORDER), cv::Point(BORDER+FIELD_WIDTH, BORDER+FIELD_HEIGHT)));
  ros::init(argc, argv, "mcl2");
  ros::NodeHandle nh;
  boost::thread ros_thread = boost::thread(boost::bind(&MCL::callbackThread, this));

  expLines.insert(expLines.begin(), vertical_field_lines.begin(), vertical_field_lines.end());
  expLines.insert(expLines.end(), horizontal_field_lines.begin(), horizontal_field_lines.end());

  // Initialize windows for debug
  // cv::namedWindow("field_points");
  // cv::namedWindow("field_lines");

  robot_pos = cv::Point3d(0,0,0);

  cf = 0; N_Particle = 100;

  std::string path = "data/bh_dist.bin"; //field with size 1 line width
  field_weight.loadData(path);
  // cv::imshow("data", test_field);
  field_weight.field = field;

  std::random_device x_rd, y_rd, w_rd;
  std::uniform_real_distribution<double> x_rgen(-FIELD_WIDTH/2,FIELD_WIDTH/2), y_rgen(-FIELD_HEIGHT/2,FIELD_HEIGHT/2), w_rgen(0,359);

  for(int i = 0; i < N_Particle; i++)
    particles.push_back(Particle(x_rgen(x_rd), y_rgen(y_rd), w_rgen(w_rd), 1/N_Particle));

  mgauss_w = 2; mgauss_x = 1; mgauss_y = 1;
  mcl_fieldpts_var = 1; mcl_wslow = mcl_wfast = mcl_aslow = mcl_afast = 0;

  scanArea.clear();
  displayed_cv = false;
  std::string config_path = "config/config.yaml";
  loadConfig(config_path);
}

void MCL::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());
  printf("노드를 선언한다!!\n");
  mcl2_subs = nh.subscribe("/kudos_vision_local_sensor_data", 1, &MCL::submitCallback, this);
  mcl2_pub = nh.advertise<mcl2::kudos_vision_mcl2_local_result>("/kudos_vision_mcl2_local_result", 1);
  while (nh.ok())
  {
    ros::spinOnce();
    usleep(1000);
  }
}

void MCL::submitCallback(const mcl2::kudos_vision_local_sensor_data::ConstPtr& msg)
{
  op3_local_mode = msg->op3_local_mode;
  int debug_num = msg->debug_num;
  if(op3_local_mode == true)
  {
    for(int i=0; i<100; i++)
    {
      sensor_data_x[i] = msg->sensor_data_x[i];
      sensor_data_y[i] = msg->sensor_data_y[i];
    }
  }
  int s_x = msg->start_point_x;
  int s_y = msg->start_point_y;
  int s_o = msg->start_point_orien;

  if(op3_local_mode == true)
  {
    if(start_point_x != s_x || start_point_y != s_y || start_point_orien != s_o)
    {
      start_point_x = s_x;
      start_point_y = s_y;
      start_point_orien = s_o;
      mcl_wslow = 0.7;
      mcl_wfast = 0.7;
      N_Particle = NDEF_Particle;
      Particles new_list;
      for(int j = 0; j < N_Particle; j++)
      {
        reset_particle = false;
        float tmp_point_x = (msg->start_point_x_list)[j];
        float tmp_point_y = (msg->start_point_y_list)[j];
        float tmp_point_orien = (msg->start_point_orien_list)[j];
        printf("(%lf %lf %lf)\n", tmp_point_x, tmp_point_y, tmp_point_orien);
        new_list.push_back(std::make_tuple(tmp_point_x, tmp_point_y, tmp_point_orien, 1/N_Particle));
      }
      particles = new_list;
      x(mean_estimate) = start_point_x;
      y(mean_estimate) = start_point_y;
      w(mean_estimate) = start_point_orien;
      publishParticles(particles, mean());
    }
  }

  diff_limit_wslow_wfast = msg->diff_limit_wslow_wfast;
  custom_local_range_distance = msg->xy_distribution;
  custom_local_range_orien = msg->orien_distribution;
}


/**
 * @brief MCL::loadConfig load config from YAML
 * @param[in] path path to config
 */
void MCL::loadConfig(std::string path)
{
  config_path = path;
  QVector<double> param;
  param.resize(13);
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(config_path.c_str());
  } catch (const std::exception& e)
  {
    std::cout <<"Fail to load yaml file." << std::endl;
  }

  param[0] = mgauss_x = doc["mgauss_x"].as<double>();
  param[1] = mgauss_y = doc["mgauss_y"].as<double>();
  param[2] = mgauss_w = doc["mgauss_w"].as<double>();
  param[3] = vgauss_x = doc["vgauss_x"].as<double>();
  param[4] = vgauss_y = doc["vgauss_y"].as<double>();
  param[5] = mcl_afast = doc["mcl_afast"].as<double>();
  param[6] = mcl_aslow = doc["mcl_aslow"].as<double>();
  param[7] = useAdaptiveParticle = doc["useAdaptiveParticle"].as<bool>();
  param[8] = mcl_heading_var = doc["mcl_heading_var"].as<double>();
  param[9] = mcl_fieldpts_var = doc["mcl_fieldpts_var"].as<double>();
  param[10] = mcl_pt2line_var = doc["mcl_pt2line_var"].as<double>();
  param[11] = mcl_ptangle_var = doc["mcl_ptangle_var"].as<double>();
  param[12] = mcl_line2line_var = doc["mcl_line2line_var"].as<double>();
  emit publishParam(param);

}

/**
 * @brief MCL::saveConfig save config to YAML
 */
void MCL::saveConfig()
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(config_path.c_str());
  } catch (const std::exception& e)
  {
    std::cout <<"Fail to load yaml file." << std::endl;
  }

  doc["mgauss_x"] = mgauss_x;
  doc["mgauss_y"] = mgauss_y;
  doc["mgauss_w"] = mgauss_w;
  doc["vgauss_x"] = vgauss_x;
  doc["vgauss_y"] = vgauss_y;
  doc["mcl_afast"] = mcl_afast;
  doc["mcl_aslow"] = mcl_aslow;
  doc["mcl_heading_var"] = mcl_heading_var;
  doc["mcl_fieldpts_var"] = mcl_fieldpts_var;
  doc["useAdaptiveParticle"] = useAdaptiveParticle;
  doc["mcl_pt2line_var"] = mcl_pt2line_var;
  doc["mcl_ptangle_var"] = mcl_ptangle_var;
  doc["mcl_line2line_var"] = mcl_line2line_var;

  // output to file
  std::ofstream fout(config_path.c_str());
  fout << doc;
}

/**
 * @brief MCL::setFeatures set features to be used
 * True -> Use field lines
 * False -> Use field points
 *
 * @param state
 */
void MCL::setFeatures(bool state)
{
  field_lines = state;
}

/**
 * @brief MCL::updateOdometry [SLOT] get the robot displacement
 * @param x displacement in X
 * @param y displacement in Y
 * @param deg displacement in degree
 */
void MCL::updateOdometry(double x, double y, double deg)
{
  motion_delta.x = x;
  motion_delta.y = y;
  motion_delta.z = deg;

  //update the motion model
  updateMotion();
}

/**
 * @brief MCL::updatePose update global robot pose as reference
 * @param x
 * @param y
 * @param deg
 */
void MCL::updatePose(double x, double y, double deg)
{
  robot_pos.x = x;
  robot_pos.y = y;
  robot_pos.z = deg;
}

/**
 * @brief MCL::setScanPoints set line segments reference for searching intersection
 * @param scanPoints vector of LineSegments
 */
void MCL::setScanPoints(std::vector<std::pair<cv::Point, cv::Point> > scanPoints)
{
  scan_Points = scanPoints;
  LineScanning();

}

/**
 * @brief MCL::setNoise set vision and motion noises
 * @param param
 */
void MCL::setNoise(QVector<double> param)
{
  mgauss_x = param[0]; mgauss_y = param[1]; mgauss_w = param[2];
  vgauss_x = param[3]; vgauss_y = param[4];
  saveConfig();
}

/**
 * @brief MCL::setMCLParam set the mcl parameters
 * @param param QVector of paramaters
 */
void MCL::setMCLParam(QVector<double> param)
{
  mcl_afast = param[0];
  mcl_aslow = param[1];
  mcl_heading_var = param[2];
  useAdaptiveParticle = param[3];
  mcl_fieldpts_var = param[4];
  mcl_pt2line_var = param[5];
  mcl_ptangle_var = param[6];
  mcl_line2line_var = param[7];

  saveConfig();
}

/**
 * @brief MCL::resetMCL reset the particle by randomly distribute them
 * @param status
 */
void MCL::resetMCL(bool status)
{

  std::random_device xrd, yrd, wrd;
  std::uniform_real_distribution<double> xrg(-450,450), yrg(-300,300), wrg(0,360);
  for(auto& p : particles)
  {
    x(p) = xrg(xrd);
    y(p) = yrg(yrd);
    w(p) = wrg(wrd);
  }

}

/**
 * @brief MCL::heading_err calculate the particle heading error
 * Particles heading error w.r.t robot's known heading
 * @param angle particle's heading in degree
 * @return weight from exp value
 */
double MCL::heading_err(double angle)
{
  if(angle>=360)
    angle -= 360;
  if(angle < 0)
    angle += 360;

  return exp(fabs(angle - robot_pos.z)/2*mcl_heading_var*mcl_heading_var);
}

/*
 * updateMotion()
 * update the particle motion model from odometry
 */
/**
 * @brief MCL::updateMotion update motion model using displacement and gaussian
 */
void MCL::updateMotion()
{
  std::random_device xrd, yrd, wrd;
  std::normal_distribution<> xgen(0.0,mgauss_x), ygen(0.0,mgauss_y), wgen(0.0,mgauss_w);

  double dx = motion_delta.x;// * motion_alpha;
  double dy = motion_delta.y;// * motion_beta;
  double dw = motion_delta.z; // * motion_gamma;

  for(auto& p : particles)
  {

    double c = cos(w(p) * DEGREE2RADIAN);
    double s = sin(w(p) * DEGREE2RADIAN);

    x(p) += c*dx - s*dy + xgen(xrd);
    y(p) += s*dx + c*dy + ygen(yrd);
    w(p) += dw + wgen(wrd);

    if(w(p)>=360)
      w(p) -= 360;
    if(w(p) < 0)
      w(p) += 360;
  }

}

/**
 * LineScanning()
 * Simple line scanning algorithm to recreate the radial line scanning
 * from a pair of points. The algorithm will use the cv::LineIterator to
 * iterate along a line to find any white points intersect with known field lines.
 *
 * Use simple hough line to find line segments
 * Remember that OpenCV Mat axis
 * o-------- x+
 * |
 * |
 * | y+
 *
 * And our world model
 * | y+
 * |
 * |
 * o-------- x+
 *
 * so please check every rotation frame
 */

void MCL::LineScanning()
{
  cv::Mat alpha = cv::Mat::zeros(field.size(), CV_8UC1);
  cv::Mat mask = cv::Mat::zeros(field.size(), CV_8UC1);// = cv::Mat::zeros(field.size(), CV_8UC1);
  cv::Mat drawing = cv::Mat::zeros(field.size(), CV_8UC3);// = cv::Mat::zeros(field.size(), CV_8UC1);

  if(vision_debug)
  {
    field.copyTo(alpha);
  }
  std::vector<QPointF> linePoints;
  std::vector<SensorData> linePoints_;
  std::vector<LineSegment> rawLines;
  std::vector<LineSegment> filteredLines;
  std::vector<LineSegment> vertical_lines, horizontal_lines;
  std::vector<LineSegment> fieldLines;


  double c = cos(robot_pos.z * DEGREE2RADIAN);
  double s = sin(robot_pos.z * DEGREE2RADIAN);

  /// Find line intersection points from reference radial segments in fov
  if(!field_lines)
  {
    for(int i = 0; i < scan_Points.size(); i++)
    {
      cv::LineIterator it(field, scan_Points[i].first, scan_Points[i].second, 8);

      cv::Point first = scan_Points[i].first;
      cv::Point second =  scan_Points[i].second;

      if(vision_debug)
        cv::line(alpha, first, second, cv::Scalar(255));

      double slope = double(second.y - first.y) / double(second.x - first.x);
      if(slope == -INFINITY || slope > 200)
        continue;
      /*
      for(int i = 0; i < it.count; i++, ++it)
        if(field.at<uchar>(it.pos()) == 255)
        {
          //Return the point(x,y) respect to robot origin
          double dx = it.pos().x - (CENTERX + robot_pos.x);
          double dy = (CENTERY - robot_pos.y) - it.pos().y;
          double point_x = c*dx + s*dy;
          double point_y = -s*dx + c*dy;
          if(vision_debug)
          {
            double world_x = c*point_x-s*point_y+(CENTERX + robot_pos.x);
            double world_y = (CENTERY - robot_pos.y) - (s*point_x+c*point_y);
            if(field_weight.distance_matrix[800*int(world_x)+int(world_y)] < 0.3)
              cv::circle(alpha, cv::Point(world_x, world_y), 5, cv::Scalar(255));
            cv::circle(alpha, cv::Point(point_x, point_y), 10, cv::Scalar(255));
            cv::circle(alpha, cv::Point((CENTERX + robot_pos.x), (CENTERY - robot_pos.y)), 2, cv::Scalar(255));
          }
          //linePoints.push_back(QPointF(point_x, point_y));
          //linePoints_.push_back(std::make_pair(point_x, point_y));  
        }
      */
      for(int i=0; i<100; i++)
      {
        bool Is_valid_data = true;
        if(sensor_data_x[i] == -100 && sensor_data_y[i] == -100)
          Is_valid_data = false;
        if(Is_valid_data == true)
        {
          linePoints.push_back(QPointF(sensor_data_x[i], sensor_data_y[i]));
          linePoints_.push_back(std::make_pair(sensor_data_x[i], sensor_data_y[i]));
        }
      }
    }
    emit publishPoints(linePoints);
  }
  /*
  else
  {
    /// Find field lines segment in the fov
    std::vector<std::vector<cv::Point> > temp;
    temp.push_back(scanArea);

    std::vector<std::vector<cv::Point> >hull( temp.size() );
    for( int i = 0; i < temp.size(); i++ )
      cv::convexHull( cv::Mat(temp[i]), hull[i], false );

    // Draw contours + hull results
    for( int i = 0; i< temp.size(); i++ )
    {
      cv::drawContours( mask, hull, i, cv::Scalar(255), cv::FILLED);
    }

    cv::bitwise_and(field, mask, line_result);

    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    cv::HoughLinesP(line_result, linesP, 1, CV_PI/180, 50, 50); // runs the actual detection

    std::random_device r_d;
    std::mt19937 gen(r_d());
    std::normal_distribution<> xgen(0,vgauss_x), ygen(0,vgauss_y);

    for( size_t i = 0; i < linesP.size(); i++ )
    {
      cv::Vec4i l = linesP[i];
      rawLines.push_back(LineSegment(l[0], l[1], l[2], l[3]));
      cv::line( drawing, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3);

      //Return the point(x,y) respect to robot origin
      double p1x = l[0] - (CENTERX + robot_pos.x)+xgen(gen);
      double p1y = (CENTERY - robot_pos.y) - l[1]-ygen(gen);
      double p2x = l[2] - (CENTERX + robot_pos.x)+xgen(gen);
      double p2y = (CENTERY - robot_pos.y) - l[3]-ygen(gen);
      double point1_x = c*p1x + s*p1y;
      double point1_y = -s*p1x + c*p1y;
      double point2_x = c*p2x + s*p2y;
      double point2_y = -s*p2x + c*p2y;

      fieldLines.push_back(LineSegment(point1_x, point1_y, point2_x, point2_y));
    }

    emit publishLines(fieldLines);
  }
  */

  if(vision_debug && field_lines)
  {
    std::vector<QPointF> clsPnts;
    for(auto line_f : expLines)
    {
      cv::Point pt_f = closestPointinLine(line_f, cv::Point(CENTERX + robot_pos.x, CENTERY - robot_pos.y), true);
      clsPnts.push_back(QPointF(pt_f.x-CENTERX, CENTERY-pt_f.y));
    }
    emit publishClsPnts(clsPnts);
    clsPnts.clear();
  }

  if(vision_debug)
  {
    displayed_cv = true;
    if(!field_lines)
    {
      cv::imshow("field_points", alpha);
    }
    else
    {
      cv::imshow("field_lines", drawing);
    }
    cv::waitKey(1);
  }
  else if (displayed_cv && !vision_debug)
  {
      displayed_cv = false;
//      cv::destroyAllWindows();
  }


  obs_Lines = rawLines;

  if(!vision_debug)
  {
      if(!field_lines)
      {
          updatePerceptionPoints(linePoints_);
      }
      else
      {
          updatePerceptionLines(rawLines);
      }
  }

}

/**
 * @brief MCL::updatePerceptionPoints measurement model update using points in lines
 * Calculate the particle weight based on Euclidean distance matrix.
 * Ref: Robust and Real-time Self-Localization Based on Omnidirectional Vision for Soccer Robots
 *
 * @param linePoints
 */
void MCL::updatePerceptionPoints(std::vector<SensorData> linePoints)
{
  int num_points = linePoints.size();
  double sum_weight = 0;
  double w_avg = 0;
  double highest = 0;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> xgen(0,vgauss_x), ygen(0,vgauss_y), wgen(0,5);

  for(auto &p : particles)
  {
    double prob= 1;
    double cmps_prob = 0;
    if(num_points !=0 )
    {
      reset_particle = false;
      for(auto &d : linePoints)
      {
        double angle_rad = (w(p)) * DEGREE2RADIAN;
        double c = cos(angle_rad);
        double s = sin(angle_rad);
        double x_d = x(d) + xgen(gen);
        double y_d = y(d) + ygen(gen);
        double world_x = (c*x_d-s*y_d)+x(p)+CENTERX;
        double world_y = (CENTERY+y(p))+(s*x_d+c*y_d);
        double distance = 1;
        if((world_x >= 0) && (world_x <= MATWIDTH) && (world_y >= 0) && (world_y <= MATHEIGHT))
          distance = field_weight.distance_matrix[800*int(world_x)+int(world_y)];
        prob *= exp((-distance*distance)/(2*mcl_fieldpts_var*mcl_fieldpts_var));
      }

      cmps_prob = 1/heading_err(w(p));

      weight(p) = prob;
      if(use_heading)
        weight(p)*=cmps_prob;
      sum_weight +=weight(p);
    }

    else
    {
      reset_particle = false;//bh: 원래 true였는데 false로 바꿈 
    }

  }

  for(auto& p : particles)
  {
    w_avg += (weight(p)/N_Particle);
    weight(p) /= sum_weight;

  }

  /// Augmented MCL variables
  mcl_wslow += mcl_aslow*(w_avg - mcl_wslow);
  mcl_wfast += mcl_afast*(w_avg - mcl_wfast);
  printf("mcl_aslow: %lf, mcl_afast: %lf\n", mcl_aslow, mcl_afast);
  printf("w_avg: %lf, mcl_wslow: %lf, mcl_wfast: %lf\n", w_avg, mcl_wslow, mcl_wfast);

  lowVarResampling();

}

/**
 * @brief MCL::p2pDistance Distance between 2 points
 * @param p1 first point cv::Point
 * @param p2 second point cv::Point
 * @return
 */
double MCL::p2pDistance(cv::Point p1, cv::Point p2)
{
  LineSegment l(p1, p2);
  return l.lineLength();
}

/**
 * @brief MCL::pointInLine check if a point is in a line segment
 * If a Point(p) is inside a Line(p1, p2), the distance between
 * p->p1 + p->p2 == Line.length() or p1 -> p2
 *
 * @param line
 * @param p
 * @return
 */
double MCL::pointInLine(LineSegment line, cv::Point p)
{

  double p1_d = p2pDistance(line.p1, p);
  double p2_d = p2pDistance(line.p2, p);
  return p1_d + p2_d;
}

/**
 * @brief MCL::lineInline Check if an observed line is parallel with the reference line
 * This helps to ignore similar features but has a different length.
 * The observed line segment should be parallel with the reference line
 * so the weight should be near zero.
 *
 * @param[in] obs Observed line segment
 * @param[in] ref Reference line segment
 * @return the probability weight
 */
double MCL::lineInline(LineSegment obs, LineSegment ref)
{
  double d_p1 = pointInLine(ref, obs.p1);
  double d_p2 = pointInLine(ref, obs.p2);

  double ref_length = ref.lineLength();

  return (fabs(d_p1 - ref_length)+fabs(d_p2-ref_length))/100;
}

/**
 * @brief MCL::closestPointinLine Computing the closest point on a Line(p1,p2) to a point
 * Ref: https://diego.assencio.com/?index=ec3d5dfdfc0b6a0d147a656f0af332bd
 * @param[in] l line segment L(p1,p2)
 * @param[in] p reference point (p)
 * @param[in] cut find the point inside the line segment [p1,p2] or along the line of p1 and p2
 * @return cv::Point closest point in Line
 */
cv::Point MCL::closestPointinLine(LineSegment &l, cv::Point p, bool cut = false)
{
  cv::Point A = l.p1;
  cv::Point v = l.p2 - l.p1;
  cv::Point2d u = p-A;
  double vu = v.x * u.x + v.y * u.y;
  double vv = v.x*v.x + v.y*v.y;

  double t = vu/vv;

  if(cut)
    t = clip(t, 0, 1);

  return A+t*v;
}

/**
 * @brief MCL::angle2Points calculate the angle between 2 points
 * @param A first cv::Point
 * @param B second cv::Point
 * @return angle in Radian
 */

double MCL::angle2Points(cv::Point A, cv::Point B)
{
  return atan2(A.y-B.y, A.x-B.x);
}

/**
 * @brief MCL::updatePerceptionLines measurement model update using line segments
 * Calculate the particle weight based on lines similarity
 *
 * @param obsLines vector of LineSegments
 */
void MCL::updatePerceptionLines(std::vector<LineSegment> obsLines)
{
  double sum_weight = 0;
  double w_avg = 0;

  for(auto& p: particles)
  {
    double p_weight = 1;
    cv::Point particle_pos = cv::Point(CENTERX+x(p), CENTERY-y(p));

    cv::Point r_pos = cv::Point(CENTERX+robot_pos.x, CENTERY-robot_pos.y);
    double angle_r = robot_pos.z *DEGREE2RADIAN;

    double cmps_prob = 1/heading_err(w(p));
    double rd = 0; double s_d = 0; double rt = 0; double s_t = 0;
    double ld = 0; double l_d = 0;

    for(auto& d_line: obsLines)
    {
      cv::Point pt = closestPointinLine(d_line, r_pos, false);
      double dist = d_line.distancePoint(r_pos);
      double angle = (angle2Points(pt, r_pos)-angle_r)*10;

      for(auto line_f: expLines)
      {
        cv::Point pt_f = closestPointinLine(line_f, particle_pos, false);
        double dist_f= line_f.distancePoint(particle_pos);
        double angle_f =(angle2Points(pt_f, particle_pos)-angle_r)*10;

        rd = fabs(dist - dist_f);
        s_d = exp((-rd*rd)/(2*mcl_pt2line_var*mcl_pt2line_var));

        rt = fabs(angle - angle_f);
        s_t = exp((-rt*rt)/(2*mcl_ptangle_var*mcl_ptangle_var));

        ld = lineInline(d_line, line_f);
        l_d = exp((-ld*ld)/(2*mcl_line2line_var*mcl_line2line_var));
        p_weight += (s_d*s_t*l_d);

      }

    }

    weight(p) = p_weight;

    if(use_heading)
      weight(p)*=cmps_prob;
    sum_weight += weight(p);
  }

  for(auto& p : particles)
  {
    w_avg += (weight(p)/N_Particle);
    weight(p) /= sum_weight;

  }

  /// Augmented MCL variables
  mcl_wslow += mcl_aslow*(w_avg - mcl_wslow);
  mcl_wfast += mcl_afast*(w_avg - mcl_wfast);

  lowVarResampling();
}

/**
 * @brief MCL::lowVarResampling apply the low variance resampling
 * Also apply the adaptive number of particles
 * Ref:
 * Ref: Monte Carlo Localization Based on Gyrodometry and Line-Detection
 */
void MCL::lowVarResampling()
{
  Particles new_list;
  std::default_random_engine rd;
  std::mt19937 gen(rd());
  std::random_device x_rd, y_rd, w_rd;
  std::uniform_real_distribution<double> rg(0.0,1.0/N_Particle), xrg(-450,450), yrg(-300,300), wrg(0,359);
  double reset_prob = std::max(0.0, 1-(mcl_wfast/mcl_wslow));
  double r = rg(rd);
  double c = weight(particles[0]);
  best_estimate = particles[0];
  int id = 0;

  double mean_x = 0;
  double mean_y = 0;
  double sin_ = 0, cos_ = 0;
  double orien = 0;

  printf("mcl_wslow/mcl_wfast: %lf\n", mcl_wslow/mcl_wfast);
  for(int j = 0; j < N_Particle; j++)
  {
    /// Add random pose to Xt for augmented mcl
    double random = ((double) rand() / (RAND_MAX));
    //if(random < reset_prob || reset_particle)
    //{
    //  reset_particle = false;
    //  new_list.push_back(std::make_tuple(xrg(x_rd), yrg(y_rd), wrg(w_rd), 1/N_Particle));
    //}
    // 랜덤으로 위치를 찾는 파티클이 그저 아무데나 가는 것 보다는
    // 특정 수치까지는 그래도 주변에서 찾아볼려는 노력을 하는게 좋지 않을까 싶어 만들었다
    // 조절 수치: mcl_wfast와 mcl_wslow의 차이
    // 주변 발견 능력치: (mcl_wfast와 mcl_wslow의 차이)*10*kudos_vision_local설정 범위*uniform 처리
    if(random < reset_prob || reset_particle)
    {
      reset_particle = false;
      if(((mcl_wslow/mcl_wfast) < diff_limit_wslow_wfast) && mcl_wslow>mcl_wfast)
      {
        double x_min = local_result_x - ((mcl_wslow/mcl_wfast)*custom_local_range_distance);
        double x_max = local_result_x + ((mcl_wslow/mcl_wfast)*custom_local_range_distance);
        double y_min = local_result_y - ((mcl_wslow/mcl_wfast)*custom_local_range_distance);
        double y_max = local_result_y + ((mcl_wslow/mcl_wfast)*custom_local_range_distance);
        double orien_min = local_result_orien - ((mcl_wslow/mcl_wfast)*custom_local_range_orien);
        double orien_max = local_result_orien + ((mcl_wslow/mcl_wfast)*custom_local_range_orien);
        if(x_min < -450)
          x_min = -450.0;
        if(x_max > 450)
          x_max = 450.0;
        if(y_min<-300)
         y_min = -300.0;
        if(y_max>300)
          y_max = 300.0;
        if(orien_min<-180)
          orien_min = -180;
        if(orien_max>180)
          orien_max = 180;
        std::uniform_real_distribution<double> cus_xrg(x_min,x_max), cus_yrg(y_min,y_max), cus_wrg(orien_min,orien_max);
        new_list.push_back(std::make_tuple(cus_xrg(x_rd), cus_yrg(y_rd), cus_wrg(w_rd), 1/N_Particle));
      }
      else
      {
        new_list.push_back(std::make_tuple(xrg(x_rd), yrg(y_rd), wrg(w_rd), 1/N_Particle));
      }
    }
    /////////customed_by_bh
    else
    {
      /// Low variance resampling
      double U = r+((double)(j)/N_Particle);
      while(U > c)
      {
        id +=1;
        c += weight(particles[id]);
      }
      if(weight(particles[id]) > weight(best_estimate))
        best_estimate = particles[id];

      new_list.push_back(particles[id]);
    }
  }

  particles = new_list;


  /// Variables for adaptive number of particles
  double dist = 0;
  double x_best = x(best_estimate);
  double y_best = y(best_estimate);
  double cf=0;

  for(auto &p : particles)
  {
    mean_x += x(p);
    mean_y += y(p);
    double deg = w(p) * M_PI/180;
    sin_ += sin(deg);
    cos_ += cos(deg);
    weight(p) = 1/N_Particle;

    double x_ = x_best-x(p);
    double y_ = y_best-y(p);
    dist += sqrt(x_*x_ + y_*y_);
  }

  mean_x /= N_Particle;
  mean_y /= N_Particle;
  orien = atan2(sin_,cos_)*180/M_PI;

  ///////////////////////////////////////////////////////////////////////////////////////////SP Point!
  local_result_x = mean_x;
  local_result_y = mean_y;
  local_result_orien = orien;

  if(op3_local_mode == true)
  {
    mcl2::kudos_vision_mcl2_local_result message;
    message.local_result_x = local_result_x;
    message.local_result_y = local_result_y;
    message.local_result_orien = local_result_orien;
    mcl2_pub.publish(message);
  }
  
  x(mean_estimate) = mean_x;
  y(mean_estimate) = mean_y;
  w(mean_estimate) = orien;

  /// Adaptive number of particles
  /// Ch. 3, Sec. D) Adaptation of the number of particles
  if(useAdaptiveParticle)
  {
    cf = (dist/N_Particle)/1081.6653;
    if(cf >= 0.08)
      N_Particle = cf*NMAX_Particle;
    else
      N_Particle = NMIN_Particle;
  }
  else
    N_Particle = NDEF_Particle;

  if(field_lines)
  {
    std::vector<QPointF> obsPnts;
    for(auto d_line : obs_Lines)
    {
      cv::Point pt_f = closestPointinLine(d_line, cv::Point(CENTERX + mean_x, CENTERY - mean_y), false);
      obsPnts.push_back(QPointF(pt_f.x-CENTERX, CENTERY-pt_f.y));
    }
    emit publishObsPnts(obsPnts);
    obsPnts.clear();
  }

  publishParticles(particles, mean());
}

/**
 * @brief MCL::FieldMatrix::FieldMatrix initialize the Euclidean's Look Up Table
 */
MCL::FieldMatrix::FieldMatrix()
{
  this->distance_matrix = new double[MATHEIGHT*MATWIDTH];
  field = cv::Mat::zeros(800, 1100, CV_64FC1);
}

/**
 * @brief MCL::FieldMatrix::loadData Load from matlab binary data
 *
 * @param path path to Look Up Table
 */
void MCL::FieldMatrix::loadData(std::string path)
{
  std::ifstream file;
  file.open(path.c_str(), std::ios::in | std::ios::binary);
  file.read((char*)distance_matrix,sizeof(double)*MATWIDTH*MATHEIGHT);
  file.close();

  for(int row = 0; row < field.rows; row++)
    for(int col = 0; col < field.cols; col++)
    {
      //int list_len = 800*col+row;
      //printf("listlen=%d\n", list_len);
      // printf("%d\n", value);
      field.at<double>(row,col) = distance_matrix[800*col+row];
    }
  //cv::imshow("Date", test_field);
  //while(char(cv::waitKey(0)) != 'q');
}
