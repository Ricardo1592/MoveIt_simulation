"""
Constantes: Poses, Markers e Med_Markers
Cada constante vai ser chamada na criação do banco de dados, 
para facilitar que os dados que vão ser inseridos no banco de dados
possam ser gerenciados mais facilmente.

POSES: Uma lista de tuplas na ordem: 
description_pose, pose_x, pose_y, pose_z, pose_r, pose_p, pose_yl
description_pose como str e os outros como float

O (cod_pose) está como autoincrement, o id de cada pose será a ordem na lista


MARKERS (AKA. ARUCO): Lista de tuples referente ao id de cara marcador, com a seguinte ordem: 
cod_medicine (Int foreign key), 
cod_pose* (int foreign key),
quadrant (int), 
x_tam_med(float), 
y_tam_med(float), 
z_tam_med(float)

* O (cod_pose) está como autoincrement, o cod_pose será a ordem na lista de POSES

"""
POSES = [
    ("quadrante1", 88.112, 327.125, 138.133, 236.530, 84.188, 94.708),
    ("quadrante2", 120.659, 327.020, 115.204, 335.203, 75.334, 62.472),
    ("quadrante3", 29.912, 328.817, 141.917, 240.411, 83.758, 93.638),
    ("quadrante4", 52.41, 325.34, 118.28, 325.36, 73.07, 70.14),
    ("quadrante5", 328.119, 324.928, 133.002, 236.064, 79.148, 97.272),
    ("quadrante6", 356.199, 321.580, 116.217, 327.198, 78.107, 70.137),
    ("quadrante7", 269.163, 320.593, 128.365, 238.034, 81.834, 96.603),
    ("quadrante8", 295.354, 330.131, 118.076, 328.100, 70.972, 64.968),
    ("drop", 149.106, 301.190, 57.375, 268.44, 302.820, 84.343),
    (
        "drop_prototipo",
        0.22207,
        0.40132,
        0.05755,
        -2.62604,
        177.13246,
        1.34048,
    ),
    ("drop_safe", 70.73, 312.596, 36.748, 268.172, 267.992, 69.122),
    ("pos_drop", 268.70, 330.18, 73.45, 86.18, 0, 4.45),
    ("home", 10.291, 42.895, 106.288, 267.739, 332.335, 92.869),
    ("pos_home", 8.789, 340.573, 144.698, 269.934, 71.139, 90.942),
    ("pos_rec", 261, 346.287, 68.557, 271.954, 269.231, 256.788),
]

MARKERS = [
    (86567, 1, 1, 0.0375, 0.02, 0.028, "BOTTLE", "AUTOMATIC"),
    (1037, 1, 1, 0.086, 0.035, 0.05, "BOTTLE", "AUTOMATIC"),
    (1434, 3, 3, 0.059, 0.03, 0.042, "BOTTLE", "AUTOMATIC"),
    (1922, 3, 3, 0.067, 0.025, 0.035, "BOTTLE", "AUTOMATIC"),
    (2358, 4, 4, 0.15, 0.05, 0.043, "BOTTLE", "AUTOMATIC"),
    (1220, 4, 4, 0.055, 0.035, 0.030, "BOTTLE", "AUTOMATIC"),
    (3227, 6, 6, 0.059, 0.02, 0.03, "BOTTLE", "AUTOMATIC"),
    (1944, 6, 6, 0.05, 0.026, 0.04, "BOTTLE", "AUTOMATIC"),
    (85818, 8, 8, 0.065, 0.04, 0.035, "BOTTLE", "AUTOMATIC"),
]
