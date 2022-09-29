from scipy import interpolate, signal
import numpy as np


# Resampling
def resample(x, y):
    flinear = interpolate.interp1d(x, y)
    #fcubic = interpolate.interp1d(time, accz, kind='cubic')
    x_int = np.arange(x[0], x[-1], 0.002)
    y_int = flinear(x_int)
    return [x_int, y_int]


def subtraction_mean(y, y_window):
    ## Subtract the arithmetic mean of vector y_tot from vector y_window and return results in y_mean
    ## Inputs:
    # y -> list containing values for calculating the mean
    # y_window -> list containing values that will be subtracted by mean
    ## Output:
    # y_mean -> list containing values of y_window subtracted by mean
    #    
    
    mean_y = sum(y)/len(y)
    y_mean = np.array(y_window) - mean_y

    return y_mean

def g_to_ms2(x):
    # Convert data from g (gravity) to m/s²
    g = 9.8 # 9.80665
    x_ms2 = []
    for i in x:
        x_ms2.append(i*g)
    
    return x_ms2

def integrate(x, y, initial = 0, threshold=0):
    # Integração do gráfico dado pelas listas x e y
    # Trapezoidal rule
    # threshold (optional) is a value for y in which the integration is considered 0
    ## Output: 
    # t -> resulting list in x axis
    # v -> resulting list in y axis

    v = []
    t = []

    soma = initial
    # First point
    v.append(soma)
    t.append(x[0])

    
    for i in range(1,len(x)):
        if (abs(y[i])>threshold):
            soma += (x[i]-x[i-1])*((y[i])+(y[i-1]))/2
        v.append(soma)
        t.append(x[i])
    
    return [t, v]


def bind_last_point(x, y):
    # Bind last point of list x to 0.
    # First point is fixed and remaining points move proportionally

    # Straight line equation:
    a = ((y[-1]-y[0])) / (x[-1]-x[0])
    b = (y[-1]) - a * x[-1]

    y_out = []
    for i in range(len(x)):
        y_out.append(y[i] - (a*x[i]+b))

    #print(a*x[0]+b)
    #print(a*x[-1]+b)

    return [y_out, a, b]


def lowpass_filter(input, fs, cutoff, numtap=130):
    fnorm = cutoff/(fs/2)
    filtered = signal.firwin(numtap, fnorm)
    #result = signal.lfilter(filtered, 1, input)
    result = signal.filtfilt(filtered, 1, input)
    return result

def lowpass_iir_filter(input, fs, cutoff, order=6):
    fnorm = cutoff/(fs/2)
    b, a = signal.iirfilter(order, fnorm, btype='lowpass')
    result = signal.filtfilt(b, a, input)
    return result

def bandpass_filter(input, fs, cutoff1, cutoff2, numtap=131):
    fnorm = [cutoff1/(fs/2), cutoff2/(fs/2)]
    filtered = signal.firwin(numtap, fnorm)
    #result = signal.lfilter(filtered, 1, input)
    result = signal.filtfilt(filtered, 1, input)
    return result

def bandpass_iir_filter(input, fs, cutoff1, cutoff2, order=4):
    fnorm = [cutoff1/(fs/2), cutoff2/(fs/2)]
    #fnorm = [2*np.pi*cutoff1, 2*np.pi*cutoff2]
    b, a = signal.iirfilter(order, fnorm)
    #result = signal.lfilter(filtered, 1, input)
    result = signal.filtfilt(b, a, input)
    return result

def find_cross(x, y, y_cross, direction):
    # Find the values of vector x in which vector y crosses the value y_cross
    # for a given direction (signal raising or falling)
    ## Inputs:
    # x -> x data vector
    # y -> y data vector
    # y_cross -> reference cross value (scalar)
    # direction -> 'r' for raising, or 'f' for falling
    ## Output
    # cross -> list of values of x when y crosses y_cross for a given direction

    cross = []
    if direction=='r':
        for i in range(1, len(y)):
            if y[i] >= y_cross and y[i-1] < y_cross:
                cross.append(x[i])
    elif direction=='f':
        for i in range(1, len(y)):
            if y[i] <= y_cross and y[i-1] > y_cross:
                cross.append(x[i])
    return cross

def find_peaks(x, sp, direction, type):
    # Find the indices of all peaks in x vector for a given direction
    ## Inputs:
    # x -> data vector
    # sp -> index of the start point 
    # direction -> direction of search ('r' for right, 'l' for left)
    # type -> type of peak to search ('max' or 'min')
    ## Output:
    # peaks -> indices of peak positions

    peaks = []
    d = np.diff(np.array(x))
    
    if direction=='r':  # right direction
        i = range(sp, len(x))
    else:               # left direction
        i = range(0, sp)

    status_atual = 'stable'
    status_anterior = 'stable'
    for j in i:
        if j>0:
            dx = x[j]-x[j-1]
            
            if dx > 0:  # subindo
                status_anterior = status_atual
                status_atual = 'raising'
            elif dx < 0: # descendo
                status_anterior = status_atual
                status_atual = 'droping'
            # Quando dx==0 mantém o mesmo status anterior
                        
            if type=='max':
                if status_atual == 'droping' and status_anterior == 'raising':
                    peaks.append(j)
            if type=='min':
                if status_atual == 'raising' and status_anterior == 'droping':
                    peaks.append(j)
            
    return peaks


def remove_outliers(x, tol):
    # Remove points from list x whose difference to the neighbor is greater than tol.
    # tol is given in %. 
    # The removed point is substituted by the neighbor point value.
    # The returned list is the same list at the input.
    for i in range(1,len(x)):
        if (x[i-1]!=0):
            if abs(x[i]-x[i-1]) > tol:
                x[i] = x[i-1]
            
    return x

def x_to_index(x, t):
    # Return the index of first element in list x that is greater than t
    for i in range(len(x)):
        if x[i] > t:
            return i

def mean(x):
    # Return the average value of vector x
    sum = 0
    for i in range(len(x)):
        sum = sum + x[i]
    return sum/len(x)

def std(x):
    # Return the standard deviation of vector x
    media = mean(x)
    d = []
    for i in range(len(x)):
        d.append((x[i]-media)**2)
    md = mean(d)
    return md**(1/2)
    
def find_point_proximity(x, y, x0, ycross, direction):
    # Find the value of ycross for a given direction in the proximity of x0 in the signal given by (x, y).
    # The value can be at right or at left of x0 and will be returned the nearest point.
    ## Inputs:
    # x -> x data
    # y -> y data
    # x0 -> starting searching point in vector x
    # ycross -> value of y data the be found
    # direction -> raising ('r') or falling ('f')
    ## Output:
    # xcross -> value of x data corresponding to ycross

    index_x0 = x_to_index(x, x0)
    right_x_value = None
    left_x_value = None
    index_right_x_value = None
    index_left_x_value = None
    xcross = None
    if direction == 'r':  # raising
        for i in range(index_x0,len(x)):
            if y[i]>=ycross and y[i-1]<=ycross:
                right_x_value = x[i]
                break
        for i in range(index_x0,0,-1):
            if y[i]>=ycross and y[i-1]<=ycross:
                left_x_value = x[i]
                break
    elif direction == 'f':  # falling
        for i in range(index_x0,len(x)):
            if y[i]<=ycross and y[i-1]>ycross:
                right_x_value = x[i]
                break
        for i in range(index_x0):
            if y[i]<=ycross and y[i-1]>ycross:
                left_x_value = x[i]
                break
    
    index_x0 = x_to_index(x, x0)
    if right_x_value != None:
        index_right_x_value = x_to_index(x, right_x_value)
    if left_x_value != None:
        index_left_x_value = x_to_index(x, left_x_value)

    if index_right_x_value != None:
        if index_left_x_value != None:
            difright = right_x_value - x0
            difleft = x0 - left_x_value
            if difleft > difright:
                xcross = right_x_value
            else:
                xcross = left_x_value
        else: 
            xcross = right_x_value
    else:
        if index_left_x_value != None:
            xcross = left_x_value
        else:
            xcross = None
    
    return xcross

   
def find_max_proximity(x, y, x0, direction):
    # Find the next value of maximum y in the proximity of x0 in the signal given by (x, y) for a given direction.
    # The value can be at right, at left or any (nearest) of x0.
    ## Inputs:
    # x -> x data
    # y -> y data
    # x0 -> starting searching point in vector x
    # direction -> 'r': right; 'l': left; 'a': any (nearest)
    ## Output:
    # [xmax, ymax] -> value of point corresponding to nearest maximum

    index_x0 = x_to_index(x, x0)
    right_x_value = None
    left_x_value = None
    index_right_x_value = None
    index_left_x_value = None
    xmax = None
    ymax = None

    # Direita
    for i in range(index_x0+1,len(y)-1):
        if y[i]>=y[i-1] and y[i]>y[i+1]:
            right_x_value = x[i]
            break
    # Esquerda
    for i in range(index_x0-1,1,-1):
        if y[i]>=y[i-1] and y[i]>y[i+1]:
            left_x_value = x[i]
            break
    
    if right_x_value != None:
        index_right_x_value = x_to_index(x, right_x_value)
    if left_x_value != None:
        index_left_x_value = x_to_index(x, left_x_value)

    if direction == 'l':
        xmax = left_x_value
    elif direction == 'r':
        xmax = right_x_value
    elif direction == 'a':
        if index_right_x_value != None:
            if index_left_x_value != None:
                difright = right_x_value - x0
                difleft = x0 - left_x_value
                if difleft > difright:
                    xmax = right_x_value
                else:
                    xmax = left_x_value
            else: 
                xmax = right_x_value
        else:
            if index_left_x_value != None:
                xmax = left_x_value
            else:
                xmax = None
    
    ymax = y[x_to_index(x, xmax)]
    return [xmax, ymax]

def rms(x):
    # Calculate root mean square value of vector x
    # soma = 0
    # for i in x:
    #     soma = soma + i*i
    # media = soma/len(x)
    # return np.sqrt(media)
    return(np.sqrt(np.mean(np.array(x)**2)))

