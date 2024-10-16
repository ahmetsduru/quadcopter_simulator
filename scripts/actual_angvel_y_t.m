function actual_angvel_y_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'states.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve a_angvel_y (16. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    a_angvel_y = data{:, 16};      % 16. kolon: a_angvel_y

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(a_angvel_y)
        figure;
        plot(timestamp, a_angvel_y, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('a_angvel_y');
        title('a_angvel_y vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
