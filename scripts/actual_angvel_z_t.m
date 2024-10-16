function actual_angvel_z_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve a_angvel_z (17. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    a_angvel_z = data{:, 17};      % 17. kolon: a_angvel_z

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(a_angvel_z)
        figure;
        plot(timestamp, a_angvel_z, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('a_angvel_z');
        title('a_angvel_z vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
