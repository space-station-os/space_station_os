import numpy as np
import matplotlib.pyplot as plt
import glob


def load_csv_as_dict(filepath):
    # ヘッダー行を取得
    with open(filepath, 'r', encoding='utf-8') as f:
        header = f.readline().strip().split(',')

    # データ読み込み
    data = np.genfromtxt(filepath, delimiter=',', skip_header=1)

    # 各列を辞書に格納（列名 → numpy配列）
    data_dict = {col: data[:, i] for i, col in enumerate(header)}
    return data_dict, header


def draw_motion_radius(in_df):
    eci_x_sr = in_df['gnc/pos_eci_x']
    eci_y_sr = in_df['gnc/pos_eci_y']
    eci_z_sr = in_df['gnc/pos_eci_z']

    eci_r_sr = np.sqrt(eci_x_sr**2 + eci_y_sr**2 + eci_z_sr**2)
    print(eci_x_sr)
    
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(eci_r_sr)
    ax.set_ylabel('Motion Radius [m]')
    ax.set_xlabel('simulation step')
    plt.show()


def draw_3d_orbit(in_df):
    eci_x = in_df['gnc/pos_eci_x']
    eci_y = in_df['gnc/pos_eci_y']
    eci_z = in_df['gnc/pos_eci_z']

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])

    # --- 地球を半透明の青い球体で描画 ---
    earth_radius = 6371e3  # [m]
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = earth_radius * np.outer(np.cos(u), np.sin(v))
    y = earth_radius * np.outer(np.sin(u), np.sin(v))
    z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='blue', alpha=0.3, edgecolor='none')

    # 軌道線を描画
    ax.plot(eci_x, eci_y, eci_z, color='red', linewidth=1.5)

    ax.view_init(elev=30, azim=45)
    ax.set_xlabel('ECI X [m]')
    ax.set_ylabel('ECI Y [m]')
    ax.set_zlabel('ECI Z [m]')
    ax.set_title('3D Orbit in ECI Frame')
    plt.show()


def main():
    in_filepath_list = sorted(glob.glob('./space_station_gnc/tests/orbit_dynamics/result_csv/*'))
    in_filepath = in_filepath_list[-1]

    print('Loaded file:', in_filepath)

    in_df, header = load_csv_as_dict(in_filepath)
    print(header)

    draw_3d_orbit(in_df)
    # draw_motion_radius(in_df)

if __name__ == '__main__':
    main()
