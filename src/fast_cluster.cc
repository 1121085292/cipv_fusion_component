//
// C++ standalone verion of fastcluster by Daniel Müllner
//
// Copyright: Christoph Dalitz, 2018
//            Daniel Müllner, 2011
// License:   BSD style license
//            (see the file LICENSE for details)
//


#include <vector>
#include <algorithm>
#include <cmath>


// extern "C" {
#include "cipv_fusion_component/src/fast_cluster.h"
// }
#include "cipv_fusion_component/src/fast_cluster_R_dm.hpp"

// // Code by Daniel Müllner
// // workaround to make it usable as a standalone version (without R)
// bool fc_isnan(double x) { return false; }

// extern "C" {
//
// Assigns cluster labels (0, ..., nclust-1) to the n points such
// that the cluster result is split into nclust clusters.
//
// Input arguments:
//   n      = number of observables
//   merge  = clustering result in R format
//   nclust = number of clusters
// Output arguments:
//   labels = allocated integer array of size n for result
//
  void cutree_k(int n, const int* merge, int nclust, int* labels) {

    int k,m1,m2,j,l;

    if (nclust > n || nclust < 2) {
      for (j=0; j<n; j++) labels[j] = 0;
      return;
    }

    // assign to each observable the number of its last merge step
    // beware: indices of observables in merge start at 1 (R convention)
    std::vector<int> last_merge(n, 0);
    for (k=1; k<=(n-nclust); k++) {
      // (m1,m2) = merge[k,]
      m1 = merge[k-1];
      m2 = merge[n-1+k-1];
      if (m1 < 0 && m2 < 0) { // both single observables
        last_merge[-m1-1] = last_merge[-m2-1] = k;
      }
      else if (m1 < 0 || m2 < 0) { // one is a cluster
        if(m1 < 0) { j = -m1; m1 = m2; } else j = -m2;
        // merging single observable and cluster
        for(l = 0; l < n; l++)
          if (last_merge[l] == m1)
            last_merge[l] = k;
        last_merge[j-1] = k;
      }
      else { // both cluster
        for(l=0; l < n; l++) {
          if( last_merge[l] == m1 || last_merge[l] == m2 )
            last_merge[l] = k;
        }
      }
    }

    // assign cluster labels
    int label = 0;
    std::vector<int> z(n,-1);
    for (j=0; j<n; j++) {
      if (last_merge[j] == 0) { // still singleton
        labels[j] = label++;
      } else {
        if (z[last_merge[j]] < 0) {
          z[last_merge[j]] = label++;
        }
        labels[j] = z[last_merge[j]];
      }
    }
  }

  //
  // Assigns cluster labels (0, ..., nclust-1) to the n points such
  // that the hierarchical clustering is stopped when cluster distance >= cdist
  //
  // Input arguments:
  //   n      = number of observables
  //   merge  = clustering result in R format
  //   height = cluster distance at each merge step
  //   cdist  = cutoff cluster distance
  // Output arguments:
  //   labels = allocated integer array of size n for result
  //
  void cutree_cdist(int n, const int* merge, double* height, double cdist, int* labels) {

    int k;

    for (k=0; k<(n-1); k++) {
      if (height[k] >= cdist) {
        break;
      }
    }
    cutree_k(n, merge, n-k, labels);
  }


  //
  // Hierarchical clustering with one of Daniel Muellner's fast algorithms
  //
  // Input arguments:
  //   n       = number of observables
  //   distmat = condensed distance matrix, i.e. an n*(n-1)/2 array representing
  //             the upper triangle (without diagonal elements) of the distance
  //             matrix, e.g. for n=4:
  //               d00 d01 d02 d03
  //               d10 d11 d12 d13   ->  d01 d02 d03 d12 d13 d23
  //               d20 d21 d22 d23
  //               d30 d31 d32 d33
  //   method  = cluster metric (see enum method_code)
  // Output arguments:
  //   merge   = allocated (n-1)x2 matrix (2*(n-1) array) for storing result.
  //             Result follows R hclust convention:
  //              - observabe indices start with one
  //              - merge[i][] contains the merged nodes in step i
  //              - merge[i][j] is negative when the node is an atom
  //   height  = allocated (n-1) array with distances at each merge step
  // Return code:
  //   0 = ok
  //   1 = invalid method
  //
  int hclust_fast(int n, double* distmat, int method, int* merge, double* height) {

    // call appropriate culstering function
    cluster_result Z2(n-1);
    if (method == HCLUST_METHOD_SINGLE) {
      // single link
      MST_linkage_core(n, distmat, Z2);
    }
    else if (method == HCLUST_METHOD_COMPLETE) {
      // complete link
      NN_chain_core<METHOD_METR_COMPLETE, t_float>(n, distmat, NULL, Z2);
    }
    else if (method == HCLUST_METHOD_AVERAGE) {
      // best average distance
      double* members = new double[n];
      for (int i=0; i<n; i++) members[i] = 1;
      NN_chain_core<METHOD_METR_AVERAGE, t_float>(n, distmat, members, Z2);
      delete[] members;
    }
    else if (method == HCLUST_METHOD_MEDIAN) {
      // best median distance (beware: O(n^3))
      generic_linkage<METHOD_METR_MEDIAN, t_float>(n, distmat, NULL, Z2);
    }
    else if (method == HCLUST_METHOD_CENTROID) {
      // best centroid distance (beware: O(n^3))
      double* members = new double[n];
      for (int i=0; i<n; i++) members[i] = 1;
      generic_linkage<METHOD_METR_CENTROID, t_float>(n, distmat, members, Z2);
      delete[] members;
    }
    else {
      return 1;
    }

    int* order = new int[n];
    if (method == HCLUST_METHOD_MEDIAN || method == HCLUST_METHOD_CENTROID) {
      generate_R_dendrogram<true>(merge, height, order, Z2, n);
    } else {
      generate_R_dendrogram<false>(merge, height, order, Z2, n);
    }
    delete[] order; // only needed for visualization

    return 0;
  }


  // Build condensed distance matrix
  // Input arguments:
  //   n  = number of observables
  //   m  = dimension of observable
  // Output arguments:
  //   out = allocated integer array of size n * (n - 1) / 2 for result
  void hclust_pdist(int n, int m, double* pts, double* out) {
    int ii = 0;
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        // Compute euclidian distance
        double d = 0;
        for (int k = 0; k < m; k ++) {
          double error = pts[i * m + k] - pts[j * m + k];
          d += (error * error);
        }
        out[ii] = d;//sqrt(d);
        ii++;
      }
    }
  }

  void cluster_points_centroid(int n, int m, double* pts, double dist, int* idx) {
    double* pdist = new double[n * (n - 1) / 2];
    int* merge = new int[2 * (n - 1)];
    double* height = new double[n - 1];

    hclust_pdist(n, m, pts, pdist);
    hclust_fast(n, pdist, HCLUST_METHOD_CENTROID, merge, height);
    cutree_cdist(n, merge, height, dist, idx);

    delete[] pdist;
    delete[] merge;
    delete[] height;
  }
// }

std::vector<int> cluster_points_centroid(std::vector<std::vector<double>> &pts, double dist)
{   
    int n = pts.size();
    int m = pts[0].size();

    // 将二维数据转换为一维连续内存块
    std::vector<double> flat_pts;
    for (const auto& row : pts) {
        flat_pts.insert(flat_pts.end(), row.begin(), row.end());
    }

    // 将数据传递给外部库进行聚类
    std::vector<int> labels(n);
    cluster_points_centroid(n, m, flat_pts.data(), std::pow(dist, 2), labels.data());
    return labels;
}
