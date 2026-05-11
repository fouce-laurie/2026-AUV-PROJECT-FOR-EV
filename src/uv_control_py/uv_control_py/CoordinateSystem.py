from math import sin,cos,atan2,asin
from uv_msgs.msg import RobotAxis

PI = 3.141592653589793238462643383279502884197169
DEG2RAD = PI/180
RAD2DEG = 180/PI

def AngleCorrect(angle ):
    if angle > 180 :
        cnt = int((angle+180)/ 360)
        angle -= cnt * 360
    if angle < -180 :
        cnt = int((angle - 180)/ 360)
        angle -= cnt * 360
    return angle

def MatrixInverse(input ):
    matrix = [[0.0 for _ in range(8)] for _ in range(4)]
    result = [0]*16
    
    # 初始化增广矩阵
    for i in range(4):
        for  j in range(4):
            matrix[i][j]     = input[i * 4 + j]
            matrix[i][j + 4] =  1.0 if (i == j) else 0.0

    # 消元过程
    for i in range(4):
        # 如果当前行的对角线元素为0，则需要进行行交换
        if matrix[i][i] == 0:
            row = i + 1
            while row < 4 and matrix[row][i] == 0: 
                row += 1

            # 如果找到了非零元素的行，则进行交换
            if row < 4:
                for j in range(8): 
                    temp = matrix[i][j]
                    matrix[i][j] = matrix[row][j]
                    matrix[row][j] = temp

        # 将对角线元素调整为1
        scale = matrix[i][i]
        for j in range(8):  
            matrix[i][j] /= scale

        # 消元
        for j in range(4):  
            if j != i:
                factor = matrix[j][i]
                for k in range(8):  
                    matrix[j][k] -= factor * matrix[i][k]
                    
    # 提取逆矩阵部分
    for i in range(4):
        for j in range(4): 
            result[i * 4 + j] = matrix[i][j + 4]
            
    return result

def MatrixMultiply(m1  ,m2 ):
    if len(m1) != 16 or len(m2) != 16:
            raise ValueError("Input matrices must be represented as 4x4 lists (length 16).")

    result = [0.0]*16

    for i in range(4):
        for j in range(4):
            result[i * 4 + j] = sum(m1[i * 4 + k] * m2[k * 4 + j] for k in range(4))

    return result

class MotionState:
    def __init__(self):
        self.vector = RobotAxis()
        self.h_matrix = [0]*16
        
    def extract(self):
        Sx = sin(self.vector.rx * DEG2RAD)
        Cx = cos(self.vector.rx * DEG2RAD)
        Sy = sin(self.vector.ry * DEG2RAD)
        Cy = cos(self.vector.ry * DEG2RAD)
        Sz = sin(self.vector.rz * DEG2RAD)
        Cz = cos(self.vector.rz * DEG2RAD)
        
        # 矩阵计算
        self.h_matrix[0] = Cy * Cz
        self.h_matrix[1] = Sx * Sy * Cz - Cx * Sz
        self.h_matrix[2] = Cx * Sy * Cz + Sx * Sz
        self.h_matrix[3] = self.vector.x

        self.h_matrix[4] = Cy * Sz
        self.h_matrix[5] = Sx * Sy * Sz + Cx * Cz
        self.h_matrix[6] = Cx * Sy * Sz - Sx * Cz
        self.h_matrix[7] = self.vector.y

        self.h_matrix[8]  = -Sy
        self.h_matrix[9]  = Sx * Cy
        self.h_matrix[10] = Cx * Cy
        self.h_matrix[11] = self.vector.z

        self.h_matrix[12] = 0.0
        self.h_matrix[13] = 0.0
        self.h_matrix[14] = 0.0
        self.h_matrix[15] = 1.0
        
    def r_extract(self):
        self.vector.x = self.h_matrix[3]
        self.vector.y = self.h_matrix[7]
        self.vector.z = self.h_matrix[11]
        
        rx1 = rx2 = 0.0
        ry1 = ry2 = 0.0
        
        if self.h_matrix[8] != 1.0 or self.h_matrix[8] != -1.0: # 若越界，放弃更新
    
            ry1 = -asin(self.h_matrix[8]) 
            ry2 = PI - ry1 

            rx1 = atan2(self.h_matrix[9] / cos(ry1), self.h_matrix[10] / cos(ry1)) 
            rx2 = atan2(self.h_matrix[9] / cos(ry2), self.h_matrix[10] / cos(ry2)) 

            if ry1 < PI / 2 and ry1 > -PI / 2:
                self.vector.ry = ry1 * RAD2DEG
            if ry2 < PI / 2 and ry2 > -PI / 2:
                self.vector.ry = ry2 * RAD2DEG

            if rx1 < PI / 2 and rx1 > -PI / 2:
                self.vector.rx = rx1 * RAD2DEG
            if rx2 < PI / 2 and rx2 > -PI / 2:
                self.vector.rx = rx2 * RAD2DEG

            # if (rz1 < PI and rz1 > -PI)
            self.vector.rz = atan2(self.h_matrix[4] / cos(self.vector.ry * DEG2RAD),
                                   self.h_matrix[0] / cos(self.vector.ry * DEG2RAD)) * RAD2DEG 
            # if (rz2 < PI and rz2 > -PI)
            #     self.vector.rz = rz2 * RAD2DEG 
    
class CoordinateSystems:
    def __init__(self):
        self.base = MotionState()
        self.target_inbase = MotionState()        
        self.target_inworld = MotionState()    
        
    def world2base(self):
        self.base.extract()
        self.target_inworld.extract()
        
        mat =  MatrixInverse(self.base.h_matrix)
        self.target_inbase.h_matrix = MatrixMultiply(mat,self.target_inworld.h_matrix)
        
        self.target_inbase.r_extract()
        
        return self.target_inbase
    
    def base2world(self):
        self.base.extract()
        self.target_inbase.extract()
        
        self.target_inworld.h_matrix = MatrixMultiply(self.base.h_matrix,self.target_inbase.h_matrix)
        
        self.target_inworld.r_extract()
        
        return self.target_inworld
        
def Cs_Move(raw ,move):
    raw.extract()
    move.extract()
    
    result = MotionState()
    
    result.h_matrix = MatrixMultiply(move.h_matrix,raw.h_matrix)
    
    result.r_extract()
    
    return result
    
def Cs_Back(raw ,back):
    raw.extract()
    back.extract()
    
    result = MotionState()
    
    mat = MatrixInverse(back.h_matrix)
    result.h_matrix = MatrixMultiply(mat,raw.h_matrix)
    
    result.r_extract()
    
    return result
