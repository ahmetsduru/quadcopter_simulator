function actual_plot_3d_xyz(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % x (2. kolon), y (3. kolon) ve z (4. kolon) verilerini ayırma
    x = data{:, 2};  % 2. kolon: x verisi
    y = data{:, 3};  % 3. kolon: y verisi
    z = data{:, 4};  % 4. kolon: z verisi

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