function plot_pos_x_y_z_t(log_folder)
    % Dosya yollarını belirleme
    actual_file_path = fullfile(log_folder, 'states.txt');
    ref_file_path = fullfile(log_folder, 'ref_pose.txt');

    % Verileri okuma
    actual_data = readtable(actual_file_path, 'Delimiter', ',');
    ref_data = readtable(ref_file_path, 'Delimiter', ',');

    % Timestamp ve ivme verilerini ayırma
    actual_timestamp = actual_data{:, 1};  % 1. kolon: timestamp
    actual_pos_x = actual_data{:, 2};      % 8. kolon: actual_pos_x
    actual_pos_y = actual_data{:, 3};      % 9. kolon: actual_pos_y
    actual_pos_z = actual_data{:, 4};     % 10. kolon: actual_pos_z

    ref_timestamp = ref_data{:, 1};        % 1. kolon: timestamp
    ref_pos_x = ref_data{:, 2};            % 8. kolon: ref_pos_x
    ref_pos_y = ref_data{:, 3};            % 9. kolon: ref_pos_y
    ref_pos_z = ref_data{:, 4};           % 10. kolon: ref_pos_z

    % Timestamp verilerini normalize etme
    if ~isempty(actual_timestamp) && ~isempty(ref_timestamp)
        actual_timestamp = actual_timestamp - actual_timestamp(1); % İlk timestamp'i çıkar
        ref_timestamp = ref_timestamp - ref_timestamp(1);          % İlk timestamp'i çıkar
    end

    % Eğer veriler doğruysa, subplot grafikleri çizelim
    if ~isempty(actual_timestamp) && ~isempty(actual_pos_x) && ~isempty(actual_pos_y) && ~isempty(actual_pos_z) ...
            && ~isempty(ref_timestamp) && ~isempty(ref_pos_x) && ~isempty(ref_pos_y) && ~isempty(ref_pos_z)
        
        figure;

        % İlk subplot: a_acc_x ve ref_pos_x vs. Normalized Timestamp
        subplot(3, 1, 1);
        plot(actual_timestamp, actual_pos_x, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_pos_x, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Position x (m)');
        title('Desired and Actual Position-Time');
        legend('Actual Pos. x', 'Desired Pos. x');
        grid on;
        hold off;

        % İkinci subplot: a_acc_y ve ref_pos_y vs. Normalized Timestamp
        subplot(3, 1, 2);
        plot(actual_timestamp, actual_pos_y, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_pos_y, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Position y (m)');
        title('Desired and Actual Position-Time');
        legend('Actual Pos. y', 'Desired Pos. y');
        grid on;
        hold off;

        % Üçüncü subplot: a_acc_z ve ref_pos_z vs. Normalized Timestamp
        subplot(3, 1, 3);
        plot(actual_timestamp, actual_pos_z, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_pos_z, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Position z (m)');
        title('Desired and Actual Position-Time');
        legend('Actual Pos. z', 'Desired Pos. z');
        grid on;
        hold off;

    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
