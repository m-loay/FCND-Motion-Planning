import numpy as np
'''
Collinearity Check¶
Collinearity for any three points can be determined easily by taking the determinant of a matrix containing 
the points.
'''
class Collinear:
    '''
    classdocs
    '''
    # Define a simple function to add a z coordinate of 1
    @staticmethod
    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)
    '''
    General Case¶
    Define a function to determine collinearity using the np.linalg.det() function.
    Introduce the epsilon threshold to allow a tolerance for collinearity.
    If the determinant is less than epsilon then the points are collinear.
    '''
    @staticmethod
    def collinearity_float(p1, p2, p3, epsilon=1e-2): 
        collinear = False
        # TODO: Add a third dimension of z=1 to each point
        # TODO: Create the matrix out of three points
        mat = np.vstack((Collinear.point(p1), Collinear.point(p2), Collinear.point(p3)))
        # TODO: Calculate the determinant of the matrix. 
        det = np.linalg.det(mat)
        # TODO: Set collinear to True if the determinant is less than epsilon
        if(det <epsilon):
            collinear = True

        return collinear

    '''
    Integer Case
    Define a function to take three points and test for collinearity by evaluating the determinant using the 
    simplified version for the 2D case:

    𝑑𝑒𝑡=𝑥1(𝑦2−𝑦3)+𝑥2(𝑦3−𝑦1)+𝑥3(𝑦1−𝑦2)
    '''
    @staticmethod
    def collinearity_check_int(p11, p22, p33): 
        collinear = False
        p1 = np.array([p11[0][0], p11[0][1]])
        p2 = np.array([p22[0][0], p22[0][1]])
        p3 = np.array([p33[0][0], p33[0][1]])
        # TODO: Calculate the determinant of the matrix using integer arithmetic
        det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1]) 
        # TODO: Set collinear to True if the determinant is equal to zero
        if det == 0:
            collinear = True
        return collinear

    # We're using collinearity here, but you could use Bresenham as well!
    @staticmethod
    def prune_path(path):
        pruned_path = [p for p in path]
        # TODO: prune the path!
        
        i = 0
        while i < len(pruned_path) - 2:
            p1 = Collinear.point(pruned_path[i])
            p2 = Collinear.point(pruned_path[i+1])
            p3 = Collinear.point(pruned_path[i+2])
            
            # If the 3 points are in a line remove the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if Collinear.collinearity_check_int(p1, p2, p3):
                # Something subtle here but we can mutate
                # `pruned_path` freely because the length
                # of the list is check on every iteration.
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        return pruned_path

# Define Points (feel free to change these)
# By default these will be cast as int64 arrays

p1 = np.array([1, 2])
p2 = np.array([2, 3])
p3 = np.array([3, 4])



'''
Test it and time it
'''
def main():
    import time
    t1 = time.time()
    collinear = Collinear.collinearity_float(p1, p2, p3)
    t_3D = time.time() - t1

    t1 = time.time()
    collinear = Collinear.collinearity_int(p1, p2, p3)
    t_2D = time.time() - t1
    t = t_3D/t_2D
    print(t)
    
if __name__ == "__main__":
    main()