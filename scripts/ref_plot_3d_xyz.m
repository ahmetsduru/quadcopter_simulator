function ref_plot_3d_xyz(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'ref_pose.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % x (5. kolon), y (6. kolon) ve z (7. kolon) verilerini ayırma
    x = data{:, 5};  % 5. kolon: x verisi
    y = data{:, 6};  % 6. kolon: y verisi
    z = data{:, 7};  % 7. kolon: z verisi

    % Eğer veriler doğruysa, 3 boyutlu grafiği çizelim
    if ~isempty(x) && ~isempty(y) && ~isempty(z)
        figure;
        plot3(x, y, z, 'LineWidth', 1.5);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title('3D Plot of X, Y, Z');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
