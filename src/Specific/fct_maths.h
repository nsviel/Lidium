#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H

/**
 * \brief Basic math functions
 */

namespace{

  //Basic functions
  float distance(glm::vec3 pt1, glm::vec3 pt2){
    //Euclidean distance
    float dist;
    //---------------------------

    dist = sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));

    //---------------------------
    return dist;
  }
  template <typename T> int get_sign(T val) {
    //Get the sign of the input -> output -1,0,1
    return (T(0) < val) - (val < T(0));
  }
  glm::vec3 centroide_vec3(std::vector<glm::vec3> XYZ){
    glm::vec3 centroid = XYZ[0];
    int size = XYZ.size();
    //---------------------------

    for(int i=0; i<size; i++){
      for(int j=0; j<3; j++){
        centroid[j] += XYZ[i][j];
      }
    }

    for(int j=0;j<3;j++){
      centroid[j] /= size;
    }

    //---------------------------
    return centroid;
  }
  template<typename Type> const Type Mean(std::vector<Type>& vec){
    float sum = 0;
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++){
      sum += vec[i];
    }

    float mean = sum / size;

    //---------------------------
    return mean;
  }
  template<typename Type> const Type fct_Mean(std::vector<Type>& vec){
    int size = vec.size();
    float sum = 0;
    //---------------------------

    for(int i=0; i<size; i++){
      sum += vec[i];
    }
    float mean = sum / size;

    //---------------------------
    return mean;
  }
  template<typename Type> std::vector<Type> InverseVector(std::vector<Type>& vec){
    //Vector inversion
    std::vector<Type> vec_out;
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++){
      vec_out.push_back(vec[size-1-i]);
    }

    //---------------------------
    return vec_out;
  }
  template<typename Type> float fct_Sum(std::vector<Type>& vec){
    //Sum of vector elements
    float out = 0;
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++){
      out = out + vec[i];
    }

    //---------------------------
    return out;
  }

  //Minimum / Maximum
  template<typename Type> const Type Max(std::vector<Type>& vec){
    Type max = vec[0];
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++)
      if(max < vec[i]) max = vec[i];

    //---------------------------
    return max;
  }
  template<typename Type> const Type fct_max(std::vector<Type>& vec){
    Type max = vec[0];
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++)
      if(max < vec[i]) max = vec[i];

    //---------------------------
    return max;
  }
  template<typename Type> const Type Max_01(std::vector<Type>& vec){
    Type max = 0;
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++){
      if(max < vec[i]) max = vec[i];
    }

    //---------------------------
    return max;
  }
  template<typename Type> const Type Max_size(std::vector<std::vector<Type>>& vec){
    Type max = vec[0].size();
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++){
      if(max < vec[i].size()) max = vec[i].size();
    }

    //---------------------------
    return max;
  }

  template<typename Type> const Type Min(std::vector<Type>& vec){
    Type min = vec[0];
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++){
      if(min > vec[i]) min = vec[i];
    }

    //---------------------------
    return min;
  }
  template<typename Type> const Type fct_min(std::vector<Type>& vec){
    Type min = vec[0];
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++){
      if(min > vec[i]) min = vec[i];
    }

    //---------------------------
    return min;
  }
  template<typename Type> const Type Min_01(std::vector<Type>& vec){
    Type min = 1;
    int size = vec.size();
    //---------------------------

    for(int i=0; i<size; i++){
      if(min > vec[i]) min = vec[i];
    }

    //---------------------------
    return min;
  }

  glm::vec2 Min_vec2(std::vector<glm::vec2> XY){
    glm::vec2 min = XY[0];
    int size = XY.size();
    //---------------------------

    for(int i=0; i<size; i++){
      for(int j=0; j<2; j++){
        if(XY[i][j] <= min[j]) min[j] = XY[i][j];
      }
    }

    //---------------------------
    return min;
  }
  glm::vec2 Max_vec2(std::vector<glm::vec2> XY){
    glm::vec2 max = XY[0];
    int size = XY.size();
    //---------------------------

    for(int i=0; i<size; i++){
      for(int j=0; j<2; j++){
        if(XY[i][j] >= max[j]) max[j] = XY[i][j];
      }
    }

    //---------------------------
    return max;
  }
  glm::vec3 Min_vec3(std::vector<glm::vec3> XYZ){
    glm::vec3 min = XYZ[0];
    int size = XYZ.size();
    //---------------------------

    for(int i=0; i<size; i++){
      for(int j=0; j<3; j++){
        if ( XYZ[i][j] <= min[j] ) min[j] = XYZ[i][j];
      }
    }

    //---------------------------
    return min;
  }
  glm::vec3 Max_vec3(std::vector<glm::vec3> XYZ){
    glm::vec3 max = XYZ[0];
    int size = XYZ.size();
    //---------------------------

    for(int i=0; i<size; i++){
      for(int j=0; j<3; j++){
        if ( XYZ[i][j] >= max[j] ) max[j] = XYZ[i][j];
      }
    }

    //---------------------------
    return max;
  }

  //Normalization
  template<typename Type> std::vector<Type> Normalize(std::vector<Type>& vec){
    std::vector<Type> vec_out(vec);
    int size = vec.size();
    //-----------------------------

    //Retrieve min & max
    Type min = vec[0];
    Type max = vec[0];
    for(int i=0; i<size; i++){
      if(vec[i] > max) max = vec[i];
      if(vec[i] < min && vec[i] >= 0) min = vec[i];
    }

    //Normalization
    for(int i=0; i<size; i++){
      vec_out[i] = (vec[i] - min) / (max - min);
    }

    //-----------------------------
    return vec_out;
  }
  template<typename Type> std::vector<Type> Normalize_01(std::vector<Type>& vec){
    std::vector<Type> vec_out(vec);
    int size = vec.size();
    //-----------------------------

    //Retrieve min & max
    Type min = 1;
    Type max = 0;
    for(int i=0; i<size; i++){
      if(vec[i] > max) max = vec[i];
      if(vec[i] < min && vec[i] >= 0) min = vec[i];
    }

    //Normalization
    for(int i=0; i<size; i++){
      vec_out[i] = (vec[i] - min) / (max - min);
    }

    //-----------------------------
    return vec_out;
  }

  //3D functions
  float dotProduct(glm::vec3 vec_A, glm::vec3 vec_B){
    float product = 0;
    //---------------------------

    // Loop for calculate cot product
    for(int i=0; i<3; i++){
      product = product + vec_A[i] * vec_B[i];
    }

    //---------------------------
    return product;
  }
  void crossProduct(std::vector<float>& vec_A, std::vector<float>& vec_B, std::vector<float>& vec_cross){
    //---------------------------

    vec_cross[0] = vec_A[1] * vec_B[2] - vec_A[2] * vec_B[1];
    vec_cross[1] = vec_A[2] * vec_B[0] - vec_A[0] * vec_B[2];
    vec_cross[2] = vec_A[0] * vec_B[1] - vec_A[1] * vec_B[0];

    //---------------------------
  }

  //Statistical functions
  //---> Standard deviation
  template<typename Type> float fct_std(std::vector<Type>& vec){
    float sum = 0.0, mean, Std = 0.0;
    int size = vec.size();
    //-------------------

    for(int i=0; i<size; i++){
      sum += vec[i];
    }

    mean = sum/size;
    for(int i=0; i<size; i++){
      Std += pow(vec[i] - mean, 2);
    }

    //-------------------
    return sqrt(Std / vec.size());
  }
  //---> Variance
  template<typename Type> float fct_var(std::vector<Type>& vec){
    int size = vec.size();
    //-------------------

    //Mean
    float sum = 0.0, mean;
    for(int i=0; i<size; i++){
      sum += vec[i];
    }
    mean = sum/size;

    //Variance
    float var = 0.0;
    for(int i=0; i<size; i++){
      var += pow(vec[i] - mean, 2);
    }
    var = var / size;

    //-------------------
    return var;
  }
  //---> Coefficient of variation
  template<typename Type> float fct_CV(std::vector<Type>& vec){
    float std = fct_std(vec);
    float CV = (std / Mean(vec)) * 100;

    return CV;
  }
  //---> R²
  float compute_R2(std::vector<float>& data_X, std::vector<float>& data_Y, std::vector<float>& coeffs){
    int size = data_Y.size();
    float up = 0.0f, bot = 0.0f, R_2;
    for(int i=0; i<size; i++)
    {
      if(coeffs.size() == 2) up += pow((data_Y[i] - (coeffs[1]*data_X[i] + coeffs[0])), 2);
      if(coeffs.size() == 3) up += pow((data_Y[i] - (coeffs[2]*pow(data_X[i],2) + coeffs[1]*data_X[i] + coeffs[0])), 2);
      bot += pow((data_Y[i] - Mean(data_Y)), 2);
    }
    R_2 = 1 - (up / bot);
    std::cout<<"---> R² = "<<R_2<<std::endl;

    return R_2;
  }

  //Sorting functions
  //--->Sort by order, keeping trace of indices
  // Use with for (auto i: sort_indexes(v)) {
  //                cout << v[i] << endl;
  //           }
  template <typename T> std::vector<size_t> sort_indexes(const std::vector<T> &v){
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    std::sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

    return idx;
  }
  //--->Sort by greater order, keeping trace of indices
  template <typename T> std::vector<size_t> sort_indexes_greater(const std::vector<T> &v){
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    std::sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

    return idx;
  }

}

#endif
