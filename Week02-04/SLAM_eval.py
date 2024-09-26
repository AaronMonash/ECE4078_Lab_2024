# evaluate the map generated by SLAM against the true map
import ast
import numpy as np
import json

def parse_groundtruth(fname : str) -> dict:
    with open(fname,'r') as f:
        # gt_dict = ast.literal_eval(f.readline())
        gt_dict = json.load(f)
        aruco_dict = {}
        for key in gt_dict:
            if key.startswith("aruco"):
                aruco_num = int(key.strip('aruco')[:-2])
                aruco_dict[aruco_num] = np.reshape([gt_dict[key]["x"], gt_dict[key]["y"]], (2,1))
    return aruco_dict

def parse_user_map(fname : str) -> dict:
    with open(fname, 'r') as f:
        usr_dict = ast.literal_eval(f.read())
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict["taglist"]):
            aruco_dict[tag] = np.reshape([usr_dict["map"][0][i],usr_dict["map"][1][i]], (2,1))
    return aruco_dict

def match_aruco_points(aruco0 : dict, aruco1 : dict):
    points0 = []
    points1 = []
    keys = []
    for key in aruco0:
        if not key in aruco1:
            continue
        
        points0.append(aruco0[key])
        points1.append(aruco1[key])
        keys.append(key)
    return keys, np.hstack(points0), np.hstack(points1)

def solve_umeyama2d(points1, points2):
    # Solve the optimal transform such that
    # R(theta) * p1_i + t = p2_i

    assert(points1.shape[0] == 2)
    assert(points1.shape[0] == points2.shape[0])
    assert(points1.shape[1] == points2.shape[1])


    # Compute relevant variables
    num_points = points1.shape[1]
    mu1 = 1/num_points * np.reshape(np.sum(points1, axis=1),(2,-1))
    mu2 = 1/num_points * np.reshape(np.sum(points2, axis=1),(2,-1))
    sig1sq = 1/num_points * np.sum((points1 - mu1)**2.0)
    sig2sq = 1/num_points * np.sum((points2 - mu2)**2.0)
    Sig12 = 1/num_points * (points2-mu2) @ (points1-mu1).T

    # Use the SVD for the rotation
    U, d, Vh = np.linalg.svd(Sig12)
    S = np.eye(2)
    if np.linalg.det(Sig12) < 0:
        S[-1,-1] = -1
    
    # Return the result as an angle and a 2x1 vector
    R = U @ S @ Vh
    theta = np.arctan2(R[1,0],R[0,0])
    x = mu2 - R @ mu1

    return theta, x

def apply_transform(theta, x, points):
    # Apply an SE(2) transform to a set of 2D points
    assert(points.shape[0] == 2)
    
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))

    points_transformed =  R @ points + x
    return points_transformed


def compute_rmse(points1, points2):
    # Compute the RMSE between two matched sets of 2D points.
    assert(points1.shape[0] == 2)
    assert(points1.shape[0] == points2.shape[0])
    assert(points1.shape[1] == points2.shape[1])
    num_points = points1.shape[1]
    residual = (points1-points2)
    # Finding highest rmse for unknown markers
    if num_points != 10:
        highest_residual = 0
        highest_residual_index = 0
        for i in range(num_points):
            if residual[0][i]**2 + residual[1][i]**2>highest_residual:
                highest_residual = residual[0][i]**2 + residual[1][i]**2
                highest_residual_index = i
        highest_residual_array = [[residual[0][highest_residual_index]],[residual[1][highest_residual_index]]]
        for i in range(10-num_points):
            residual = np.concatenate((residual, highest_residual_array), axis =1)
            
    unraveled_residual = residual    
    residual = residual.ravel()    
    MSE = 1.0/10 * np.sum(residual**2)
    
    return np.sqrt(MSE), unraveled_residual


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser("Matching the estimated map and the true map")
    parser.add_argument("groundtruth", type=str, help="The ground truth file name.")
    parser.add_argument("estimate", type=str, help="The estimate file name.")
    args = parser.parse_args()

    gt_aruco = parse_groundtruth(args.groundtruth)
    us_aruco = parse_user_map(args.estimate)

    taglist, us_vec, gt_vec = match_aruco_points(us_aruco, gt_aruco)


    rmse, residuals = compute_rmse(us_vec, gt_vec)
    print("The RMSE before alignment: {}".format(rmse))

    theta, x = solve_umeyama2d(us_vec, gt_vec)
    us_vec_aligned = apply_transform(theta, x, us_vec)

    print("The following parameters optimally transform the estimated points to the ground truth.")
    print("Rotation Angle: {}".format(theta))
    print("Translation Vector: ({}, {})".format(x[0,0], x[1,0]))

    rmse, residuals = compute_rmse(us_vec_aligned, gt_vec)
    print("The RMSE after alignment: {}".format(rmse))

    print()
    print("Pred Locations")
    print(taglist)
    print("Real Locations")
    print("np.array("+np.array2string(gt_vec, precision=4, separator=',')+')')
    print("Aligned Pred Locations")
    print("np.array("+np.array2string(us_vec_aligned, precision=4, separator=',')+')')
    print("Marker errors (Highest error assumed for unseen markers)")
    print("np.array("+np.array2string(residuals, precision=4, separator=',')+')')


