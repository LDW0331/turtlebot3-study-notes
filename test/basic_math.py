# -*- coding: UTF-8 -*-  
'''
    定义了机器人操作中的基础框架
'''

import math
'''
    二维空间点集
'''

class Point2D:
    def __init__(self,x=0.0,y=0.0):
        """
        初始化方法。
        
        Args:
            x (float, optional): x坐标值，默认为0.0。
            y (float, optional): y坐标值，默认为0.0。
        """
        self.x=x
        self.y=y
    def __add__(self,value):
        """
        实现加法运算符重载。允许我们自定义对象之间的加法行为。
        
        Args:
            value (Point2D): 要与之相加的另一个 Point2D 对象。
        
        Returns:
            Point2D: 返回一个新的 Point2D 对象，其 x 和 y 坐标分别等于当前对象的 x 和 y 坐标与参数对象的 x 和 y 坐标之和。
        """
        return Point2D(self.x+value.x,self.y+value.y)
    def __str__(self):
        return 'X:%f,Y:%f\n'%(self.x,self.y)
    __repr__ = __str__

'''
    二维空间无原点向量
'''
class Vector2D:

    def __init__(self, x1=0.0, y1=0.0, x2=None, y2=None):
        if x2 is not None and y2 is not None:
            self.x = x2-x1
            self.y = y2-y1
        else:
            self.x = x1
            self.y = y1
    def __add__(self,value):
        ''' 定义加法'''
        return Vector2D(0, 0, self.x + value.x, self.y + value.y)
    def __sub__(self, value):
        return Vector2D(0, 0, self.x - value.x, self.y - value.y)
    def __mul__(self,value):
        '''乘法'''
        return self.x*value.x+self.y*value.y
    def Cross(self,vec):
        '''叉乘'''
        return self.x*vec.y-self.y*vec.x
    def Theta(self,vec):
        '''获取相对全局坐标的theta值'''
        sc = self * vec   # 当前向量与 vec 的点积
        ss = self.Cross(vec)  # 当前向量与 vec 的叉积
        return math.atan2(ss, sc)
    def Formalize(self,val=1.0):
        '''标准化'''
        l=abs(self)
        if l != 0:
            self.x = self.x * val / l
            self.y = self.y * val / l
    def distance_to(self, vec):
        return math.sqrt((self.x - vec.x) ** 2 + (self.y - vec.y) ** 2)

 
    def __abs__(self):
        return math.sqrt(self.x*self.x+self.y*self.y)
    def __str__(self):
        return 'X:%f,Y:%f\n'%(self.x,self.y)
    __repr__ = __str__

'''
    二维空间带有向量的点
'''
class ArrowPoint2D(Point2D):
    def __init__(self, x1=0.0, y1=0.0, x2=None, y2=None, th=None):
        # 初始化 Point2D
        Point2D.__init__(self, x1, y1)
        
        if x2 is not None and y2 is not None:
            # 根据第二个点的坐标计算方向向量
            self.vec_x = x2 - x1
            self.vec_y = y2 - y1
        elif th is not None:
            # 根据角度 th 计算方向向量
            self.vec_x = math.cos(th)
            self.vec_y = math.sin(th)
        else:
            # 默认的向量
            self.vec_x = 0.0
            self.vec_y = 0.0

    def __abs__(self):
        return math.sqrt(self.vec_x ** 2+ self.vec_y **2)

    def __mul__(self, value):
        return self.vec_x * value.x + self.vec_y * value.y

    def Cross(self, vec):
        return self.vec_x * vec.y - self.vec_y * vec.x

    def Reload(self, x, y, th):
        Point2D.__init__(self, x, y)
        self.vec_x = math.cos(th)
        self.vec_y = math.sin(th)

    def Theta_Format(self):
        return self.x, self.y, math.atan2(self.y, self.x)

    def Len(self):
        return math.sqrt(self.vec_x * self.vec_x + self.vec_y * self.vec_y)

    def Theta(self,vec):
        '''获取相对全局坐标的theta值'''
        sc = self * vec   # 当前向量与 vec 的点积
        ss = self.Cross(vec)  # 当前向量与 vec 的叉积
        return math.atan2(ss, sc)

    def Formalize(self, l=1.0):
        length = self.Len()
        if length != 0:
            self.vec_x = (self.vec_x / length) * l
            self.vec_y = (self.vec_y / length) * l
    def distance_to(self, vec):
            return math.sqrt((self.vec_x - vec.x) ** 2 + (self.vec_y - vec.y) ** 2)


    def __str__(self):
        return 'X:%f, Y:%f, VEC: %f %f\n' % (self.x, self.y, self.vec_x, self.vec_y)

    __repr__ = __str__
    #def Affine()
def Distance(p1,p2):
    ''' 计算距离    '''
    return math.sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))

def Cos2Linear(v,max_Vel=1.0):
    '''
        将cos曲线优化为线性曲线
    '''
    if v>1: return 0
    return max_Vel*math.acos(v)/math.pi/2

def Theta2AccelLinear(theta, max_Accel, max_reverse_ratio=0.2):
    '''
        基于夹角,计算其加速度特性
    '''
    # 生成线性控制力，这里假设theta是以弧度为单位，且范围是(-pi/2, pi/2)
    control_force = (1.0 - abs(theta) / (math.pi / 2)) * 2  # 修正计算方式
    control_force *= max_Accel  # 按比例生成控制力
    
    # 限制反向加速度的最大值
    if control_force < -max_Accel * max_reverse_ratio:
        control_force = -max_Accel * max_reverse_ratio
    
    return control_force
# def Theta2AccelLinear(theta,max_Accel,max_reverse_ratio=0.2):
#     '''
#         基于夹角,计算其加速度特性
#     '''
#     f=1.0*(math.pi/2-abs(theta*2))/math.pi*2  #生成线性控制力
#     f=f*max_Accel   #按比例生成控制力
#     if f<-max_Accel*max_reverse_ratio:
#         f=-max_Accel*max_reverse_ratio
#     return f


def Theta2AccelAngluar(theta, max_angular_velocity, max_accel=10.0):
    '''
        基于夹角,计算器角加速度特性
    '''
    angular_accel = theta * max_angular_velocity / math.pi
    # 限制幅度
    angular_accel = max(min(angular_accel, max_accel), -max_accel)
    return angular_accel
# def Theta2AccelAngluar(theta,max_Rad_Per_s,max_Accel=10.0):
#     '''
#         基于夹角,计算器角加速度特性
#     '''
#     ang=theta*max_Rad_Per_s/math.pi
#     #   限制幅度
#     if ang> max_Accel:ang=max_Accel
#     if ang<-max_Accel:ang=-max_Accel
#     return ang
def Vec2Ang(v1, v2):
    '''
        计算两个向量之间的夹角
    '''
    cross_product = v1.Cross(v2)
    dot_product = v1.x * v2.x + v1.y * v2.y
    theta = math.atan2(cross_product, dot_product)
    return theta
# def Vec2Ang(v1,v2):
#     '''
#         计算两个向量之间的夹角
#     '''
#     sinTheta=v1.Cross(v2)/abs(v2)/abs(v1)
#     theta=math.asin(sinTheta)
#     flag=(v1*v2>0)  #检测夹角
#     if not flag:
#         if sinTheta>0: theta= math.pi-theta
#         else: theta= -theta-math.pi
#     return theta

if __name__=='__main__':
    a=ArrowPoint2D(x1=0.0,y1=0.0,x2=1.0,y2=1.0)
    a.Formalize() 